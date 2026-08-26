"""
Analyze a football video and generate a 0-1 control curve based only on
stadium crowd loudness.

For stereo broadcasts, the script uses the L-R side signal to attenuate
center-panned commentary and retain more stadium ambience. This is a useful
prototype approximation, not perfect source separation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
import wave
from pathlib import Path

import numpy as np


def require_program(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(f"找不到 {name}，请先安装并加入 PATH。")


def get_audio_channels(video: Path) -> int:
    require_program("ffprobe")
    result = subprocess.run(
        [
            "ffprobe", "-v", "error", "-select_streams", "a:0",
            "-show_entries", "stream=channels", "-of", "json", str(video)
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr[-1500:])
    streams = json.loads(result.stdout).get("streams", [])
    if not streams:
        raise RuntimeError("视频中没有音频轨道。")
    return int(streams[0].get("channels", 1))


def extract_audio(
    video: Path,
    wav_path: Path,
    sample_rate: int,
    requested_mode: str,
) -> str:
    require_program("ffmpeg")
    channels = get_audio_channels(video)

    if requested_mode == "auto":
        mode = "side" if channels >= 2 else "mono"
    elif requested_mode == "side" and channels < 2:
        print("警告：输入不是立体声，已回退到 mono。", file=sys.stderr)
        mode = "mono"
    else:
        mode = requested_mode

    if mode == "side":
        # L-R reduces audio located at the stereo centre, usually commentary.
        audio_filter = (
            "pan=mono|c0=0.5*c0-0.5*c1,"
            "highpass=f=120,lowpass=f=4500"
        )
    else:
        audio_filter = "highpass=f=120,lowpass=f=4500"

    print(f"[1/3] 提取音频，模式：{mode}")

    result = subprocess.run(
        [
            "ffmpeg", "-y", "-i", str(video), "-vn",
            "-af", audio_filter,
            "-ac", "1", "-ar", str(sample_rate),
            "-c:a", "pcm_s16le", str(wav_path),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr[-2000:])
    return mode


def calculate_loudness(
    wav_path: Path,
    window_ms: float,
    hop_ms: float,
) -> tuple[np.ndarray, np.ndarray]:
    print("[2/3] 计算声浪包络...")

    with wave.open(str(wav_path), "rb") as wav:
        if wav.getnchannels() != 1 or wav.getsampwidth() != 2:
            raise RuntimeError("需要单声道 16-bit PCM WAV。")

        sr = wav.getframerate()
        window = max(1, round(sr * window_ms / 1000))
        hop = max(1, round(sr * hop_ms / 1000))
        if hop > window:
            raise ValueError("hop-ms 不能大于 window-ms。")

        remainder = np.empty(0, dtype=np.float32)
        times_all: list[np.ndarray] = []
        db_all: list[np.ndarray] = []
        frame_index = 0
        chunk_frames = sr * 20

        while True:
            raw = wav.readframes(chunk_frames)
            if not raw:
                break

            chunk = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
            data = np.concatenate((remainder, chunk))

            if data.size < window:
                remainder = data
                continue

            count = 1 + (data.size - window) // hop
            frames = np.lib.stride_tricks.sliding_window_view(
                data, window
            )[::hop][:count]

            rms = np.sqrt(
                np.mean(frames * frames, axis=1, dtype=np.float64) + 1e-12
            )
            db = 20 * np.log10(rms + 1e-9)

            ids = np.arange(frame_index, frame_index + count)
            times = (ids * hop + window / 2) / sr

            times_all.append(times.astype(np.float32))
            db_all.append(db.astype(np.float32))

            frame_index += count
            remainder = data[count * hop:]

    if not db_all:
        raise RuntimeError("音频过短，无法分析。")

    return np.concatenate(times_all), np.concatenate(db_all)


def normalize(
    db: np.ndarray,
    low_percentile: float,
    high_percentile: float,
    gamma: float,
) -> tuple[np.ndarray, float, float]:
    low_db = float(np.percentile(db, low_percentile))
    high_db = float(np.percentile(db, high_percentile))
    high_db = max(high_db, low_db + 1.0)

    value = np.clip((db - low_db) / (high_db - low_db), 0.0, 1.0)
    value = np.power(value, max(0.1, gamma))
    return value.astype(np.float32), low_db, high_db


def smooth(
    values: np.ndarray,
    hop_ms: float,
    attack_ms: float,
    release_ms: float,
) -> np.ndarray:
    hop_s = hop_ms / 1000.0
    attack_alpha = 1 - math.exp(-hop_s / max(0.001, attack_ms / 1000))
    release_alpha = 1 - math.exp(-hop_s / max(0.001, release_ms / 1000))

    result = np.empty_like(values, dtype=np.float32)
    state = float(values[0])

    for index, target_value in enumerate(values):
        target = float(target_value)
        alpha = attack_alpha if target > state else release_alpha
        state += alpha * (target - state)
        result[index] = state

    return np.clip(result, 0.0, 1.0)


def save_csv(
    path: Path,
    times: np.ndarray,
    intensity: np.ndarray,
    raw_intensity: np.ndarray,
    db: np.ndarray,
) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.writer(file)
        writer.writerow(["time_s", "intensity", "raw_intensity", "crowd_db"])
        for row in zip(times, intensity, raw_intensity, db):
            writer.writerow([
                f"{float(row[0]):.3f}",
                f"{float(row[1]):.4f}",
                f"{float(row[2]):.4f}",
                f"{float(row[3]):.2f}",
            ])


def save_plot(path: Path, times: np.ndarray, intensity: np.ndarray) -> bool:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    plt.figure(figsize=(12, 4))
    plt.plot(times, intensity, linewidth=0.8)
    plt.xlabel("Time (s)")
    plt.ylabel("Crowd intensity")
    plt.ylim(0, 1.05)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="仅根据现场观众声浪生成 0-1 控制曲线。"
    )
    parser.add_argument("video", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("crowd_analysis_output"),
    )
    parser.add_argument(
        "--channel-mode",
        choices=["auto", "side", "mono"],
        default="auto",
    )
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--window-ms", type=float, default=40.0)
    parser.add_argument("--hop-ms", type=float, default=20.0)
    parser.add_argument("--low-percentile", type=float, default=10.0)
    parser.add_argument("--high-percentile", type=float, default=98.0)
    parser.add_argument("--gamma", type=float, default=0.9)
    parser.add_argument("--attack-ms", type=float, default=60.0)
    parser.add_argument("--release-ms", type=float, default=260.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    video = args.video.resolve()

    if not video.exists():
        print(f"视频不存在：{video}", file=sys.stderr)
        return 2

    if args.window_ms <= 0 or args.hop_ms <= 0:
        print("window-ms 和 hop-ms 必须大于 0。", file=sys.stderr)
        return 2

    if not 0 <= args.low_percentile < args.high_percentile <= 100:
        print("归一化分位数无效。", file=sys.stderr)
        return 2

    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)

    wav_path = output / "crowd_audio.wav"
    csv_path = output / "atmosphere.csv"
    plot_path = output / "atmosphere_preview.png"
    metadata_path = output / "analysis_metadata.txt"

    try:
        mode = extract_audio(
            video, wav_path, args.sample_rate, args.channel_mode
        )
        times, crowd_db = calculate_loudness(
            wav_path, args.window_ms, args.hop_ms
        )
        raw, low_db, high_db = normalize(
            crowd_db,
            args.low_percentile,
            args.high_percentile,
            args.gamma,
        )
        intensity = smooth(
            raw,
            args.hop_ms,
            args.attack_ms,
            args.release_ms,
        )
        save_csv(csv_path, times, intensity, raw, crowd_db)
        plot_created = save_plot(plot_path, times, intensity)

        metadata_path.write_text(
            "\n".join([
                f"video={video}",
                f"channel_mode={mode}",
                f"window_ms={args.window_ms}",
                f"hop_ms={args.hop_ms}",
                f"normalization_low_db={low_db:.2f}",
                f"normalization_high_db={high_db:.2f}",
                f"attack_ms={args.attack_ms}",
                f"release_ms={args.release_ms}",
                "scene_analysis=false",
                "speech_recognition=false",
                "keyword_events=false",
                "event_enhancement=false",
            ]),
            encoding="utf-8",
        )
    except Exception as error:
        print(f"分析失败：{error}", file=sys.stderr)
        return 1

    print("[3/3] 完成")
    print(f"声浪曲线：{csv_path}")
    print(f"分析参数：{metadata_path}")
    if plot_created:
        print(f"预览图：{plot_path}")

    if mode == "mono":
        print(
            "注意：单声道模式无法分离解说，曲线代表广播音轨总体声压。"
        )
    else:
        print(
            "已使用 L-R 侧声道降低居中解说，曲线更偏向现场环境声。"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())