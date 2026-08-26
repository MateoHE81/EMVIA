"""
Automatic football event analysis for Prototype A.

Pipeline:
1. Extract 16 kHz mono audio from the local MP4 with FFmpeg.
2. Transcribe commentary with faster-whisper.
3. Calculate audio excitement peaks.
4. Calculate coarse visual scene-change peaks.
5. Classify SHOT / FOUL / PENALTY / GOAL from commentary keywords,
   then use audio and scene evidence to adjust confidence.
6. Save transcript.csv and events.csv.

This is an automatic first-prototype parser, not a referee-grade sports
understanding model. Review events.csv before a formal user study.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import shutil
import subprocess
import sys
import wave
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np
from faster_whisper import WhisperModel

try:
    import ctranslate2
except ImportError:
    ctranslate2 = None


@dataclass
class TranscriptSegment:
    start: float
    end: float
    text: str


@dataclass
class Candidate:
    time_s: float
    event: str
    keyword_score: float
    audio_z: float
    scene_z: float
    confidence: float
    text: str


RULES: dict[str, list[tuple[str, float]]] = {
    "GOAL": [
        (r"\bgo+a+l+\b", 4.0),
        (r"\bscores?\b", 3.0),
        (r"\bhas scored\b", 3.5),
        (r"\bback of the net\b", 3.5),
        (r"球进了", 4.0),
        (r"进球", 3.5),
        (r"破门", 3.5),
        (r"得分了", 3.0),
        (r"入网", 3.0),
    ],
    "PENALTY": [
        (r"\bpenalty\b", 4.0),
        (r"\bspot kick\b", 3.5),
        (r"\bfrom the spot\b", 3.0),
        (r"点球", 4.0),
        (r"十二码", 4.0),
        (r"罚球点", 3.0),
    ],
    "FOUL": [
        (r"\bfoul\b", 3.0),
        (r"\bfree kick\b", 2.5),
        (r"\byellow card\b", 3.0),
        (r"\bred card\b", 3.5),
        (r"\bbooked\b", 2.5),
        (r"犯规", 3.0),
        (r"任意球", 2.5),
        (r"黄牌", 3.0),
        (r"红牌", 3.5),
        (r"吃牌", 2.5),
    ],
    "SHOT": [
        (r"\bshot\b", 2.5),
        (r"\bshoots?\b", 2.5),
        (r"\bsave[sd]?\b", 2.5),
        (r"\bwide\b", 1.8),
        (r"\bover the bar\b", 2.0),
        (r"\boff the post\b", 3.0),
        (r"\bwoodwork\b", 2.5),
        (r"射门", 2.8),
        (r"打门", 2.8),
        (r"扑出", 2.5),
        (r"门柱", 3.0),
        (r"横梁", 3.0),
        (r"偏出", 2.2),
        (r"高出", 2.2),
    ],
}

NEGATED_GOAL_PATTERNS = [
    r"\bno goal\b",
    r"\bnot a goal\b",
    r"\bgoal disallowed\b",
    r"\bdisallowed goal\b",
    r"进球无效",
    r"不算进球",
    r"越位在先",
]

MIN_KEYWORD_SCORE = {
    "GOAL": 2.8,
    "PENALTY": 2.8,
    "FOUL": 2.0,
    "SHOT": 1.8,
}


def ensure_executable(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(f"找不到 {name}。请先安装并把它加入系统 PATH。")


def run_ffmpeg_extract_audio(video: Path, wav_path: Path) -> None:
    ensure_executable("ffmpeg")
    command = [
        "ffmpeg", "-y",
        "-i", str(video),
        "-vn",
        "-ac", "1",
        "-ar", "16000",
        "-c:a", "pcm_s16le",
        str(wav_path),
    ]
    print("[1/4] 正在提取音频...")
    completed = subprocess.run(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"FFmpeg 提取失败：\n{completed.stderr[-2000:]}")


def robust_z(values: np.ndarray, radius: int) -> np.ndarray:
    """Rolling median/MAD robust z-score."""
    if values.size == 0:
        return values.astype(np.float32)

    output = np.zeros_like(values, dtype=np.float32)
    radius = max(3, radius)

    for i in range(values.size):
        lo = max(0, i - radius)
        hi = min(values.size, i + radius + 1)
        window = values[lo:hi]
        median = float(np.median(window))
        mad = float(np.median(np.abs(window - median)))
        scale = 1.4826 * mad + 1e-6
        output[i] = (values[i] - median) / scale

    return np.clip(output, -5.0, 12.0)


def audio_features(
    wav_path: Path,
    window_seconds: float = 0.25,
) -> tuple[np.ndarray, np.ndarray]:
    print("[2/4] 正在分析音频能量...")
    with wave.open(str(wav_path), "rb") as wav:
        channels = wav.getnchannels()
        sample_width = wav.getsampwidth()
        sample_rate = wav.getframerate()
        frame_count = wav.getnframes()
        raw = wav.readframes(frame_count)

    if channels != 1 or sample_width != 2:
        raise RuntimeError("预期为单声道 16-bit PCM WAV。")

    samples = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    size = max(1, int(sample_rate * window_seconds))
    usable = (samples.size // size) * size
    if usable == 0:
        raise RuntimeError("音频过短，无法分析。")

    blocks = samples[:usable].reshape(-1, size)
    rms = np.sqrt(np.mean(blocks * blocks, axis=1) + 1e-12)
    db = 20.0 * np.log10(rms + 1e-9)
    times = (np.arange(db.size) + 0.5) * window_seconds

    radius = int(30.0 / window_seconds)
    return times, robust_z(db, radius)


def scene_features(
    video_path: Path,
    sample_interval: float = 0.5,
) -> tuple[np.ndarray, np.ndarray]:
    print("[3/4] 正在分析镜头变化...")
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"无法打开视频：{video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps <= 0:
        fps = 25.0

    sample_step = max(1, int(round(fps * sample_interval)))
    frame_index = 0
    previous = None
    times: list[float] = []
    differences: list[float] = []

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        if frame_index % sample_step == 0:
            small = cv2.resize(frame, (320, 180))
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)

            if previous is not None:
                diff = cv2.absdiff(gray, previous)
                differences.append(float(np.mean(diff)))
                times.append(frame_index / fps)

            previous = gray

        frame_index += 1

    cap.release()

    if not differences:
        return np.array([], dtype=np.float32), np.array([], dtype=np.float32)

    values = np.asarray(differences, dtype=np.float32)
    radius = max(3, int(30.0 / sample_interval))
    return np.asarray(times, dtype=np.float32), robust_z(values, radius)


def choose_device(requested: str) -> tuple[str, str]:
    if requested == "cpu":
        return "cpu", "int8"
    if requested == "cuda":
        return "cuda", "float16"

    if ctranslate2 is not None:
        try:
            if ctranslate2.get_cuda_device_count() > 0:
                return "cuda", "float16"
        except Exception:
            pass

    return "cpu", "int8"


def transcribe_audio(
    wav_path: Path,
    model_name: str,
    requested_device: str,
    language: str | None,
) -> list[TranscriptSegment]:
    device, compute_type = choose_device(requested_device)
    print(
        f"[4/4] 正在转写解说：model={model_name}, "
        f"device={device}, compute_type={compute_type}"
    )

    model = WhisperModel(
        model_name,
        device=device,
        compute_type=compute_type,
    )

    segments_generator, info = model.transcribe(
        str(wav_path),
        language=language,
        beam_size=5,
        vad_filter=True,
        word_timestamps=True,
        condition_on_previous_text=True,
    )

    segments: list[TranscriptSegment] = []
    for segment in segments_generator:
        text = segment.text.strip()
        if text:
            segments.append(
                TranscriptSegment(
                    start=float(segment.start),
                    end=float(segment.end),
                    text=text,
                )
            )

    detected = getattr(info, "language", "unknown")
    probability = getattr(info, "language_probability", 0.0)
    print(f"识别语言：{detected}，置信度：{probability:.2f}")
    return segments


def normalize_text(text: str) -> str:
    text = text.lower()
    text = re.sub(r"[，。！？、,:;.!?()\[\]{}\"']", " ", text)
    text = re.sub(r"\s+", " ", text)
    return text.strip()


def score_event(text: str, event: str) -> float:
    normalized = normalize_text(text)
    score = 0.0

    for pattern, weight in RULES[event]:
        if re.search(pattern, normalized, flags=re.IGNORECASE):
            score += weight

    if event == "GOAL":
        for pattern in NEGATED_GOAL_PATTERNS:
            if re.search(pattern, normalized, flags=re.IGNORECASE):
                score -= 6.0

    return score


def max_feature_near(
    query_time: float,
    times: np.ndarray,
    scores: np.ndarray,
    radius_seconds: float,
) -> float:
    if times.size == 0:
        return 0.0

    mask = np.abs(times - query_time) <= radius_seconds
    if not np.any(mask):
        index = int(np.argmin(np.abs(times - query_time)))
        return float(scores[index])

    return float(np.max(scores[mask]))


def confidence_from_evidence(
    keyword_score: float,
    audio_z: float,
    scene_z: float,
) -> float:
    evidence = (
        keyword_score
        + 0.40 * max(0.0, min(audio_z, 6.0))
        + 0.25 * max(0.0, min(scene_z, 6.0))
    )
    confidence = 1.0 - math.exp(-evidence / 5.5)
    return float(np.clip(confidence, 0.05, 0.99))


def build_candidates(
    segments: Sequence[TranscriptSegment],
    audio_times: np.ndarray,
    audio_scores: np.ndarray,
    scene_times: np.ndarray,
    scene_scores: np.ndarray,
) -> list[Candidate]:
    candidates: list[Candidate] = []

    for index, segment in enumerate(segments):
        previous_text = segments[index - 1].text if index > 0 else ""
        next_text = segments[index + 1].text if index + 1 < len(segments) else ""
        context = f"{previous_text} {segment.text} {next_text}"

        event_scores = {
            event: score_event(context, event)
            for event in RULES
        }

        event = max(event_scores, key=event_scores.get)
        keyword_score = event_scores[event]

        if keyword_score < MIN_KEYWORD_SCORE[event]:
            continue

        event_time = max(0.0, (segment.start + segment.end) / 2.0 - 0.35)
        audio_z = max_feature_near(
            event_time, audio_times, audio_scores, radius_seconds=2.5
        )
        scene_z = max_feature_near(
            event_time, scene_times, scene_scores, radius_seconds=2.0
        )

        if keyword_score < 2.4 and max(audio_z, scene_z) < 0.8:
            continue

        confidence = confidence_from_evidence(
            keyword_score, audio_z, scene_z
        )

        candidates.append(
            Candidate(
                time_s=event_time,
                event=event,
                keyword_score=keyword_score,
                audio_z=audio_z,
                scene_z=scene_z,
                confidence=confidence,
                text=segment.text,
            )
        )

    return candidates


def deduplicate_candidates(candidates: Iterable[Candidate]) -> list[Candidate]:
    ordered = sorted(candidates, key=lambda item: item.time_s)
    merged: list[Candidate] = []

    for candidate in ordered:
        duplicate_index = None

        for index in range(len(merged) - 1, -1, -1):
            old = merged[index]
            if candidate.time_s - old.time_s > 10.0:
                break

            if candidate.event == old.event and abs(candidate.time_s - old.time_s) <= 8.0:
                duplicate_index = index
                break

        if duplicate_index is None:
            merged.append(candidate)
        elif candidate.confidence > merged[duplicate_index].confidence:
            merged[duplicate_index] = candidate

    goal_times = [item.time_s for item in merged if item.event == "GOAL"]
    penalty_times = [item.time_s for item in merged if item.event == "PENALTY"]

    filtered: list[Candidate] = []
    for item in merged:
        if item.event == "SHOT" and any(
            abs(item.time_s - goal_time) <= 10.0
            for goal_time in goal_times
        ):
            continue

        if item.event == "FOUL" and any(
            abs(item.time_s - penalty_time) <= 8.0
            for penalty_time in penalty_times
        ):
            continue

        filtered.append(item)

    return sorted(filtered, key=lambda item: item.time_s)


def write_transcript(
    output_path: Path,
    segments: Sequence[TranscriptSegment],
) -> None:
    with output_path.open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.writer(file)
        writer.writerow(["start_s", "end_s", "text"])
        for segment in segments:
            writer.writerow(
                [f"{segment.start:.3f}", f"{segment.end:.3f}", segment.text]
            )


def write_events(output_path: Path, events: Sequence[Candidate]) -> None:
    with output_path.open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_s",
                "event",
                "confidence",
                "keyword_score",
                "audio_z",
                "scene_z",
                "text",
            ]
        )
        for item in events:
            writer.writerow(
                [
                    f"{item.time_s:.3f}",
                    item.event,
                    f"{item.confidence:.3f}",
                    f"{item.keyword_score:.2f}",
                    f"{item.audio_z:.2f}",
                    f"{item.scene_z:.2f}",
                    item.text,
                ]
            )


def format_time(seconds: float) -> str:
    minutes = int(seconds // 60)
    remaining = seconds - minutes * 60
    return f"{minutes:02d}:{remaining:05.2f}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="自动解析足球视频并生成 Prototype A 事件时间表。"
    )
    parser.add_argument("video", type=Path, help="本地 MP4 文件")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("analysis_output"),
        help="输出目录",
    )
    parser.add_argument(
        "--model",
        default="small",
        help="faster-whisper 模型，例如 base、small、medium",
    )
    parser.add_argument(
        "--device",
        choices=["auto", "cpu", "cuda"],
        default="auto",
    )
    parser.add_argument(
        "--language",
        default="auto",
        help="auto、zh、en 等",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    video = args.video.resolve()

    if not video.exists():
        print(f"文件不存在：{video}", file=sys.stderr)
        return 2

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    wav_path = output_dir / "audio_16k_mono.wav"
    transcript_path = output_dir / "transcript.csv"
    events_path = output_dir / "events.csv"

    try:
        run_ffmpeg_extract_audio(video, wav_path)
        audio_times, audio_scores = audio_features(wav_path)
        scene_times, scene_scores = scene_features(video)

        language = None if args.language.lower() == "auto" else args.language
        segments = transcribe_audio(
            wav_path,
            args.model,
            args.device,
            language,
        )

        write_transcript(transcript_path, segments)

        candidates = build_candidates(
            segments,
            audio_times,
            audio_scores,
            scene_times,
            scene_scores,
        )
        events = deduplicate_candidates(candidates)
        write_events(events_path, events)

    except Exception as error:
        print(f"\n分析失败：{error}", file=sys.stderr)
        return 1

    print("\n自动识别结果：")
    if not events:
        print("没有检测到事件。请检查解说语言、音轨或关键词。")
    else:
        for item in events:
            print(
                f"{format_time(item.time_s)}  "
                f"{item.event:<8}  "
                f"confidence={item.confidence:.2f}  "
                f"{item.text}"
            )

    print(f"\n事件表：{events_path}")
    print(f"解说转写：{transcript_path}")
    print("正式用户测试前，请先人工快速检查 events.csv。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())