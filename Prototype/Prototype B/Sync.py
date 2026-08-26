#!/usr/bin/env python3
"""
Prototype B: Playback and Synchronization

1. Plays the match video through VLC.
2. Reads time_s and intensity from atmosphere.csv.
3. Uses VLC's real playback position as the synchronization clock.
4. Sends ATMOS,0.0000-1.0000 to the ESP32 at a fixed interval.
5. Receives DATA lines from the ESP32 and saves sensor/output logs.
6. Sends STOP when playback pauses, ends, fails, or the program exits.

Expected ESP32 protocol:
    PING
    ATMOS,0.42
    STOP

Expected ESP32 data:
    DATA,esp_ms,bpm,ir,ax,ay,az,gx,gy,gz,target,realtime,led,vib,ptc
"""

from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import serial


@dataclass(frozen=True)
class AtmosphereCurve:
    times: list[float]
    values: list[float]

    @property
    def duration(self) -> float:
        return self.times[-1]

    def value_at(self, time_s: float) -> float:
        """Return linearly interpolated intensity; zero outside the curve."""
        if time_s < 0.0 or time_s > self.times[-1] + 0.10:
            return 0.0

        if time_s <= self.times[0]:
            return self.values[0]

        index = bisect.bisect_right(self.times, time_s)

        if index >= len(self.times):
            return self.values[-1]

        t0 = self.times[index - 1]
        t1 = self.times[index]
        v0 = self.values[index - 1]
        v1 = self.values[index]

        if t1 <= t0:
            return v1

        ratio = (time_s - t0) / (t1 - t0)
        return v0 + ratio * (v1 - v0)


@dataclass
class SerialRecord:
    host_time: str
    line: str


def load_curve(csv_path: Path) -> AtmosphereCurve:
    times: list[float] = []
    values: list[float] = []

    with csv_path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)

        if not reader.fieldnames:
            raise RuntimeError("atmosphere.csv 没有表头。")

        required = {"time_s", "intensity"}
        missing = required.difference(reader.fieldnames)

        if missing:
            raise RuntimeError(
                "atmosphere.csv 缺少字段：" + ", ".join(sorted(missing))
            )

        previous_time = -1.0

        for row_number, row in enumerate(reader, start=2):
            try:
                time_s = float(row["time_s"])
                intensity = float(row["intensity"])
            except (TypeError, ValueError) as error:
                raise RuntimeError(
                    f"atmosphere.csv 第 {row_number} 行数据无效。"
                ) from error

            if time_s < previous_time:
                raise RuntimeError("atmosphere.csv 的 time_s 必须按升序排列。")

            previous_time = time_s
            times.append(time_s)
            values.append(max(0.0, min(1.0, intensity)))

    if len(times) < 2:
        raise RuntimeError("atmosphere.csv 至少需要两个数据点。")

    return AtmosphereCurve(times=times, values=values)


def configure_vlc_directory(vlc_dir: Path | None) -> None:
    """Make libvlc discoverable before importing python-vlc."""
    if os.name != "nt":
        return

    candidates: list[Path] = []

    if vlc_dir is not None:
        candidates.append(vlc_dir)

    candidates.extend(
        [
            Path(r"C:\Program Files\VideoLAN\VLC"),
            Path(r"C:\Program Files (x86)\VideoLAN\VLC"),
        ]
    )

    for candidate in candidates:
        if (candidate / "libvlc.dll").exists():
            os.environ["PATH"] = (
                str(candidate) + os.pathsep + os.environ.get("PATH", "")
            )

            if hasattr(os, "add_dll_directory"):
                os.add_dll_directory(str(candidate))

            return


def import_vlc(vlc_dir: Path | None) -> Any:
    configure_vlc_directory(vlc_dir)

    try:
        import vlc
    except (ImportError, OSError) as error:
        raise RuntimeError(
            "无法加载 VLC。请安装 VLC 与 python-vlc，并确保二者位数一致。"
        ) from error

    return vlc


def send_line(serial_port: serial.Serial, message: str) -> None:
    serial_port.write((message.rstrip() + "\n").encode("utf-8"))
    serial_port.flush()


def wait_for_ping(serial_port: serial.Serial, startup_s: float = 3.0) -> bool:
    """Wait for ESP32 reset, then verify serial communication."""
    print("等待 ESP32 启动...")
    deadline = time.monotonic() + startup_s

    while time.monotonic() < deadline:
        raw = serial_port.readline()
        if raw:
            print("[ESP32]", raw.decode("utf-8", errors="replace").strip())

    send_line(serial_port, "PING")
    deadline = time.monotonic() + 2.0

    while time.monotonic() < deadline:
        raw = serial_port.readline()

        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        print("[ESP32]", line)

        if line == "ACK,PONG":
            return True

    return False


def serial_reader(
    serial_port: serial.Serial,
    output_queue: queue.Queue[SerialRecord],
    stop_event: threading.Event,
) -> None:
    while not stop_event.is_set():
        try:
            raw = serial_port.readline()
        except serial.SerialException:
            stop_event.set()
            break

        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()

        if line:
            output_queue.put(
                SerialRecord(
                    host_time=dt.datetime.now().isoformat(
                        timespec="milliseconds"
                    ),
                    line=line,
                )
            )


def parse_data_line(line: str) -> list[str] | None:
    parts = line.split(",")

    if len(parts) != 15 or parts[0] != "DATA":
        return None

    return parts[1:]


def create_log_files(output_dir: Path) -> tuple[Any, ...]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")

    sensor_path = output_dir / f"sensor_data_{stamp}.csv"
    command_path = output_dir / f"feedback_commands_{stamp}.csv"

    sensor_file = sensor_path.open(
        "w", newline="", encoding="utf-8-sig", buffering=1
    )
    command_file = command_path.open(
        "w", newline="", encoding="utf-8-sig", buffering=1
    )

    sensor_writer = csv.writer(sensor_file)
    command_writer = csv.writer(command_file)

    sensor_writer.writerow(
        [
            "host_time",
            "video_time_s",
            "esp_ms",
            "bpm",
            "ir",
            "ax",
            "ay",
            "az",
            "gx",
            "gy",
            "gz",
            "target",
            "realtime",
            "led_pwm",
            "vib_pwm",
            "ptc_pwm",
        ]
    )

    command_writer.writerow(
        [
            "host_time",
            "video_time_s",
            "curve_intensity",
            "sent_intensity",
            "command",
        ]
    )

    return (
        sensor_file,
        command_file,
        sensor_writer,
        command_writer,
        sensor_path,
        command_path,
    )


def drain_serial_queue(
    serial_queue: queue.Queue[SerialRecord],
    sensor_writer: Any,
    player: Any,
) -> None:
    while True:
        try:
            record = serial_queue.get_nowait()
        except queue.Empty:
            break

        fields = parse_data_line(record.line)

        if fields is None:
            if record.line.startswith(("READY", "ACK", "ERROR")):
                print("[ESP32]", record.line)
            continue

        video_ms = player.get_time()
        video_time_s = max(0.0, video_ms / 1000.0) if video_ms >= 0 else 0.0

        sensor_writer.writerow(
            [record.host_time, f"{video_time_s:.3f}", *fields]
        )


def wait_until_playing(
    player: Any,
    vlc: Any,
    timeout_s: float = 10.0,
) -> None:
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        state = player.get_state()

        if state == vlc.State.Playing:
            return

        if state in (vlc.State.Error, vlc.State.Ended):
            raise RuntimeError(f"VLC 无法播放视频，当前状态：{state}")

        time.sleep(0.05)

    raise RuntimeError("VLC 启动超时。")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="播放视频，并按 atmosphere.csv 同步控制 ESP32。"
    )

    parser.add_argument("video", type=Path, help="原始比赛视频路径")
    parser.add_argument("atmosphere", type=Path, help="atmosphere.csv 路径")
    parser.add_argument("--port", required=True, help="ESP32 串口，例如 COM3")
    parser.add_argument("--baud", type=int, default=115200)

    parser.add_argument(
        "--send-interval-ms",
        type=float,
        default=20.0,
        help="ATMOS 发送间隔，默认 20 ms",
    )
    parser.add_argument(
        "--max-intensity",
        type=float,
        default=1.0,
        help="输出上限，首次 PTC 测试建议使用 0.5",
    )
    parser.add_argument(
        "--start-at",
        type=float,
        default=0.0,
        help="从视频第几秒开始",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="仅运行指定秒数，默认播放到视频结束",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("playback_sync_output"),
    )
    parser.add_argument(
        "--vlc-dir",
        type=Path,
        default=None,
        help=r"VLC 文件夹，例如 C:\Program Files\VideoLAN\VLC",
    )
    parser.add_argument(
        "--mute-video",
        action="store_true",
        help="视频静音，仅用于调试",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    video = args.video.resolve()
    atmosphere_path = args.atmosphere.resolve()

    if not video.exists():
        print(f"视频不存在：{video}", file=sys.stderr)
        return 2

    if not atmosphere_path.exists():
        print(f"声浪曲线不存在：{atmosphere_path}", file=sys.stderr)
        return 2

    if args.send_interval_ms < 10.0:
        print("发送间隔不建议低于 10 ms。", file=sys.stderr)
        return 2

    if not 0.0 <= args.max_intensity <= 1.0:
        print("max-intensity 必须在 0 到 1 之间。", file=sys.stderr)
        return 2

    if args.start_at < 0:
        print("start-at 不能为负数。", file=sys.stderr)
        return 2

    curve = load_curve(atmosphere_path)
    vlc = import_vlc(args.vlc_dir)

    print(f"已加载声浪曲线：{len(curve.times)} 个数据点")
    print(f"曲线时长：{curve.duration:.2f} 秒")

    serial_port: serial.Serial | None = None
    player = None
    reader_thread: threading.Thread | None = None
    reader_stop = threading.Event()
    serial_queue: queue.Queue[SerialRecord] = queue.Queue()

    (
        sensor_file,
        command_file,
        sensor_writer,
        command_writer,
        sensor_path,
        command_path,
    ) = create_log_files(args.output_dir.resolve())

    return_code = 0

    try:
        serial_port = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=0.05,
            write_timeout=1.0,
        )

        if not wait_for_ping(serial_port):
            raise RuntimeError(
                "没有收到 ACK,PONG。请关闭 Arduino 串口监视器并检查端口。"
            )

        reader_thread = threading.Thread(
            target=serial_reader,
            args=(serial_port, serial_queue, reader_stop),
            daemon=True,
        )
        reader_thread.start()

        instance = vlc.Instance("--no-video-title-show")
        media = instance.media_new(str(video))
        player = instance.media_player_new()
        player.set_media(media)

        if args.mute_video:
            player.audio_set_mute(True)

        print("开始播放视频...")
        player.play()
        wait_until_playing(player, vlc)

        if args.start_at > 0:
            player.set_time(round(args.start_at * 1000))
            time.sleep(0.15)

        send_interval_s = args.send_interval_ms / 1000.0
        next_send = time.monotonic()
        last_safe_state: str | None = None
        test_start_video_s = args.start_at

        print("同步运行中，按 Ctrl+C 可安全停止。")

        while True:
            state = player.get_state()
            now = time.monotonic()

            drain_serial_queue(serial_queue, sensor_writer, player)

            if reader_stop.is_set():
                raise RuntimeError("ESP32 串口连接中断。")

            if state == vlc.State.Playing:
                video_ms = player.get_time()

                if video_ms < 0:
                    time.sleep(0.005)
                    continue

                video_time_s = video_ms / 1000.0

                if (
                    args.duration is not None
                    and video_time_s - test_start_video_s >= args.duration
                ):
                    print("已达到指定测试时长。")
                    break

                if now >= next_send:
                    curve_intensity = curve.value_at(video_time_s)
                    sent_intensity = min(
                        curve_intensity,
                        args.max_intensity,
                    )

                    command = f"ATMOS,{sent_intensity:.4f}"
                    send_line(serial_port, command)

                    command_writer.writerow(
                        [
                            dt.datetime.now().isoformat(
                                timespec="milliseconds"
                            ),
                            f"{video_time_s:.3f}",
                            f"{curve_intensity:.4f}",
                            f"{sent_intensity:.4f}",
                            command,
                        ]
                    )

                    next_send += send_interval_s

                    if next_send < now - send_interval_s:
                        next_send = now + send_interval_s

                last_safe_state = None

            elif state == vlc.State.Paused:
                if last_safe_state != "paused":
                    send_line(serial_port, "STOP")
                    print("视频已暂停，反馈输出已停止。")
                    last_safe_state = "paused"
                next_send = now + send_interval_s

            elif state in (vlc.State.Ended, vlc.State.Stopped):
                print("视频播放结束。")
                break

            elif state == vlc.State.Error:
                raise RuntimeError("VLC 播放发生错误。")

            else:
                if last_safe_state != "not_playing":
                    send_line(serial_port, "STOP")
                    last_safe_state = "not_playing"
                next_send = now + send_interval_s

            time.sleep(0.003)

    except KeyboardInterrupt:
        print("\n收到停止指令。")

    except Exception as error:
        print(f"\n运行失败：{error}", file=sys.stderr)
        return_code = 1

    finally:
        if serial_port is not None and serial_port.is_open:
            try:
                send_line(serial_port, "STOP")
                time.sleep(0.05)
            except Exception:
                pass

        if player is not None:
            try:
                player.stop()
            except Exception:
                pass

        reader_stop.set()

        if reader_thread is not None:
            reader_thread.join(timeout=1.0)

        if player is not None:
            drain_serial_queue(serial_queue, sensor_writer, player)

        sensor_file.close()
        command_file.close()

        if serial_port is not None and serial_port.is_open:
            serial_port.close()

        print(f"传感器日志：{sensor_path}")
        print(f"反馈指令日志：{command_path}")
        print("已发送 STOP，反馈输出关闭。")

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())