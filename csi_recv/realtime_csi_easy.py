#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
实时 CSI 存储 + 训练采集 / 推理脚本（精简保存版）

采集：
  python .\realtime_csi_easy.py -p COM7 -b 921600 -o .\capture_run_01 --collect-training-data

推理：
  python .\realtime_csi_easy.py -p COM7 -b 921600 -o .\capture_run_01 --model-path .\csi_hr.keras

说明：
- 训练采集模式：仅保存原始 CSI_DATA 到 csi_data.csv，并保存 session_labels.json
- 推理模式：保存原始 CSI_DATA，同时加载模型做实时推理
- 不再保存 raw_serial.log / bad_lines.log / espnow_data.csv / csi_meta.csv / training_data.txt
"""

from __future__ import annotations

import argparse
import ast
import csv
import json
import os
import re
import time
from collections import deque
from datetime import datetime
from io import StringIO
from typing import Optional

import numpy as np
import serial
from scipy.signal import butter, filtfilt, savgol_filter


CSI_DATA_HEADER = [
    "type",
    "pkt_count",
    "mac",
    "rssi",
    "rate",
    "noise_floor",
    "channel",
    "rx_local_timestamp_us",
    "len",
    "data",
    "host_rx_us",
    "host_rx_iso",
]

LABEL_OPTIONS = {
    "posture": {
        "token_prefix": "P",
        "label": "姿态",
        "options": {
            "1": "坐",
            "2": "站",
            "3": "趴",
        },
    },
    "orientation": {
        "token_prefix": "O",
        "label": "方向",
        "options": {
            "1": "正对",
            "2": "侧对",
            "3": "背对",
        },
    },
    "distance": {
        "token_prefix": "D",
        "label": "距离",
        "options": {
            "1": "1m",
            "2": "3m",
            "3": "5m",
        },
    },
    "scene": {
        "token_prefix": "S",
        "label": "场景",
        "options": {
            "1": "实验室",
            "2": "走道",
        },
    },
}

SUBJECT_GENDER_OPTIONS = {
    "w": "女",
    "m": "男",
}


class Stats:
    def __init__(self) -> None:
        self.raw_lines = 0
        self.csi_data_lines = 0
        self.bad_lines = 0
        self.bad_csi_lines = 0
        self.pred_count = 0
        self.last_print_time = time.time()
        self.last_csi_count = 0

    def maybe_print(self, interval: float) -> None:
        now = time.time()
        dt = now - self.last_print_time
        if dt < interval:
            return

        csi_delta = self.csi_data_lines - self.last_csi_count
        csi_rate = csi_delta / dt if dt > 0 else 0.0

        print(
            f"[{datetime.now().strftime('%H:%M:%S')}] "
            f"saved_csi={self.csi_data_lines} ({csi_rate:.2f}/s), "
            f"pred={self.pred_count}, "
            f"bad={self.bad_lines}, bad_csi={self.bad_csi_lines}"
        )

        self.last_print_time = now
        self.last_csi_count = self.csi_data_lines


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def normalize_subject_code(value: str) -> str:
    match = re.fullmatch(r"\s*([wmWM])\s*0?([1-9]|10)\s*", value)
    if match is None:
        raise ValueError("受试者编号格式必须为 w1-w10 或 m1-m10")

    gender = match.group(1).lower()
    subject_index = int(match.group(2))
    return f"{gender}{subject_index:02d}"


def prompt_choice(label_name: str, options: dict[str, str]) -> str:
    prompt_text = ", ".join(f"{code}={name}" for code, name in options.items())
    while True:
        try:
            value = input(f"请选择{label_name} ({prompt_text})：").strip()
        except EOFError as exc:
            raise RuntimeError(
                f"当前终端无法交互输入，请用命令行参数显式传入{label_name}编码。"
            ) from exc

        if value in options:
            return value

        print(f"{label_name}输入无效，请重新输入。")


def prompt_subject_code() -> str:
    while True:
        try:
            value = input("请选择受试者编号（w1-w10 表示女，m1-m10 表示男）：").strip()
        except EOFError as exc:
            raise RuntimeError(
                "当前终端无法交互输入，请用命令行参数显式传入受试者编号。"
            ) from exc

        try:
            return normalize_subject_code(value)
        except ValueError as exc:
            print(str(exc))


def build_tagged_filename(session_id: Optional[str], filename: str) -> str:
    if not session_id:
        return filename

    parent_dir = os.path.dirname(filename)
    basename = os.path.basename(filename)
    stem, ext = os.path.splitext(basename)
    tagged_name = f"{session_id}_{stem}{ext}" if stem else f"{session_id}{ext}"
    return os.path.join(parent_dir, tagged_name) if parent_dir else tagged_name


def build_subject_output_dir(base_output_dir: str, subject_code: Optional[str]) -> str:
    normalized = os.path.normpath(base_output_dir)
    if not subject_code:
        return normalized

    return os.path.join(normalized, subject_code)


def resolve_training_session_labels(args: argparse.Namespace) -> tuple[Optional[dict[str, str]], bool]:
    if not args.collect_training_data or args.disable_session_labeling:
        return None, False

    labels: dict[str, str] = {}
    used_prompt = False

    for field_name in ("posture", "orientation", "distance", "scene"):
        option_info = LABEL_OPTIONS[field_name]
        raw_value = getattr(args, f"{field_name}_code")

        if raw_value is None:
            raw_value = prompt_choice(option_info["label"], option_info["options"])
            used_prompt = True
        else:
            raw_value = raw_value.strip()

        if raw_value not in option_info["options"]:
            raise ValueError(
                f"{option_info['label']}编码无效，可选值为：{', '.join(option_info['options'].keys())}"
            )

        labels[field_name] = raw_value

    subject_code = args.subject_code
    if subject_code is None:
        subject_code = prompt_subject_code()
        used_prompt = True
    else:
        subject_code = normalize_subject_code(subject_code)

    labels["subject"] = subject_code
    return labels, used_prompt


def build_session_id(labels: dict[str, str], session_start: datetime) -> str:
    time_token = session_start.strftime("T%Y%m%d_%H%M%S")
    return "_".join(
        [
            f"P{labels['posture']}",
            f"O{labels['orientation']}",
            time_token,
            f"D{labels['distance']}",
            f"S{labels['scene']}",
        ]
    )


def build_session_metadata(
    labels: dict[str, str],
    session_id: str,
    session_start: datetime,
    output_dir: str,
) -> dict[str, object]:
    subject_code = labels["subject"]
    subject_gender_code = subject_code[0]
    subject_index = subject_code[1:]
    time_token = session_start.strftime("T%Y%m%d_%H%M%S")

    return {
        "session_id": session_id,
        "output_dir": os.path.abspath(output_dir),
        "subject_dir_name": subject_code,
        "session_start_time": session_start.isoformat(timespec="seconds"),
        "labels": {
            "posture": {
                "token": f"P{labels['posture']}",
                "code": labels["posture"],
                "value": LABEL_OPTIONS["posture"]["options"][labels["posture"]],
            },
            "orientation": {
                "token": f"O{labels['orientation']}",
                "code": labels["orientation"],
                "value": LABEL_OPTIONS["orientation"]["options"][labels["orientation"]],
            },
            "time": {
                "token": time_token,
                "value": session_start.strftime("%Y-%m-%d %H:%M:%S"),
            },
            "distance": {
                "token": f"D{labels['distance']}",
                "code": labels["distance"],
                "value": LABEL_OPTIONS["distance"]["options"][labels["distance"]],
            },
            "scene": {
                "token": f"S{labels['scene']}",
                "code": labels["scene"],
                "value": LABEL_OPTIONS["scene"]["options"][labels["scene"]],
            },
            "subject": {
                "token": subject_code,
                "gender_code": subject_gender_code,
                "gender": SUBJECT_GENDER_OPTIONS[subject_gender_code],
                "index": subject_index,
                "value": f"{SUBJECT_GENDER_OPTIONS[subject_gender_code]}{int(subject_index)}号受试者",
            },
        },
    }


def write_session_metadata(path: str, metadata: dict[str, object]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)
        f.write("\n")


def print_session_label_summary(metadata: dict[str, object]) -> None:
    labels = metadata["labels"]
    print("本次采集标签如下：")
    print(f"  姿态   -> {labels['posture']['token']} ({labels['posture']['value']})")
    print(f"  方向   -> {labels['orientation']['token']} ({labels['orientation']['value']})")
    print(f"  时间   -> {labels['time']['token']} ({labels['time']['value']})")
    print(f"  距离   -> {labels['distance']['token']} ({labels['distance']['value']})")
    print(f"  场景   -> {labels['scene']['token']} ({labels['scene']['value']})")
    print(f"  受试者 -> {labels['subject']['token']} ({labels['subject']['value']})")
    print(f"  受试者目录 -> {metadata['subject_dir_name']}")
    print(f"  输出目录 -> {metadata['output_dir']}")


def build_stats_summary(stats: "Stats") -> dict[str, int]:
    return {
        "raw_lines": stats.raw_lines,
        "csi_data_lines": stats.csi_data_lines,
        "bad_lines": stats.bad_lines,
        "bad_csi_lines": stats.bad_csi_lines,
        "pred_count": stats.pred_count,
    }


def parse_csi_data_line(line: str) -> list[str]:
    row = next(csv.reader(StringIO(line)))
    if len(row) != 10:
        raise ValueError(f"CSI_DATA field count mismatch: {len(row)}")

    int(row[1])  # pkt_count
    int(row[3])  # rssi
    int(row[4])  # rate
    int(row[5])  # noise_floor
    int(row[6])  # channel
    int(row[7])  # rx_local_timestamp_us

    declared_len = int(row[8])
    data_list = ast.literal_eval(row[9])
    if not isinstance(data_list, list):
        raise ValueError("CSI_DATA data is not a list")
    if declared_len != len(data_list):
        raise ValueError(
            f"CSI_DATA len mismatch: declared={declared_len}, actual={len(data_list)}"
        )

    _ = [int(x) for x in data_list]
    return row


def parse_csi_amplitudes(csi_data_str: str) -> np.ndarray:
    values = ast.literal_eval(csi_data_str)
    arr = np.asarray(values, dtype=np.float32)

    if arr.size < 2:
        raise ValueError("CSI raw data too short")

    if arr.size % 2 != 0:
        arr = arr[:-1]

    imag = arr[0::2]
    real = arr[1::2]
    return np.sqrt(real * real + imag * imag)


def _safe_padlen(x: np.ndarray) -> int:
    return max(0, min(len(x) - 2, 12))


def remove_dc(signal_1d: np.ndarray, fs: float = 20.0) -> np.ndarray:
    x = np.asarray(signal_1d, dtype=np.float32)
    x = x - np.mean(x)

    if len(x) < 9:
        return x

    nyq = 0.5 * fs
    cutoff = 0.05 / nyq
    if cutoff <= 0 or cutoff >= 1:
        return x

    b, a = butter(2, cutoff, btype="highpass")
    padlen = _safe_padlen(x)
    return x if padlen <= 0 else filtfilt(b, a, x, padlen=padlen)


def butter_bandpass_filter(
    signal_1d: np.ndarray,
    lowcut: float,
    highcut: float,
    fs: float,
    order: int = 3,
) -> np.ndarray:
    x = np.asarray(signal_1d, dtype=np.float32)

    if len(x) < 9:
        return x

    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    if not (0 < low < high < 1):
        return x

    b, a = butter(order, [low, high], btype="band")
    padlen = _safe_padlen(x)
    return x if padlen <= 0 else filtfilt(b, a, x, padlen=padlen)


def savitzky_golay_smooth(
    signal_1d: np.ndarray,
    window_length: int = 15,
    polyorder: int = 3,
) -> np.ndarray:
    x = np.asarray(signal_1d, dtype=np.float32)
    n = len(x)

    if n < 5:
        return x

    wl = min(window_length, n if n % 2 == 1 else n - 1)
    if wl < 3:
        return x

    if wl <= polyorder:
        wl = polyorder + 2
        if wl % 2 == 0:
            wl += 1
        if wl > n:
            wl = n if n % 2 == 1 else n - 1

    if wl < 3 or wl <= polyorder:
        return x

    return savgol_filter(x, window_length=wl, polyorder=polyorder)


def open_csv_with_header(path: str, header: list[str]):
    exists = os.path.exists(path) and os.path.getsize(path) > 0
    f = open(path, "a", encoding="utf-8", newline="")
    writer = csv.writer(f)
    if not exists:
        writer.writerow(header)
        f.flush()
    return f, writer


def main() -> None:
    parser = argparse.ArgumentParser(description="实时读取、保存并处理 ESP32 CSI 串口输出（精简保存版）")
    parser.add_argument("-p", "--port", required=True)
    parser.add_argument("-b", "--baud", type=int, default=921600)
    parser.add_argument("-o", "--output-dir", default="./csi_capture")
    parser.add_argument("--stats-interval", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=1.0)

    parser.add_argument("--collect-training-data", action="store_true")
    parser.add_argument("--disable-session-labeling", action="store_true")
    parser.add_argument("--posture-code")
    parser.add_argument("--orientation-code")
    parser.add_argument("--distance-code")
    parser.add_argument("--scene-code")
    parser.add_argument("--subject-code")

    parser.add_argument("--model-path", default="csi_hr.keras")
    parser.add_argument("--window-size", type=int, default=100)

    parser.add_argument("--fs", type=float, default=20.0)
    parser.add_argument("--lowcut", type=float, default=0.8)
    parser.add_argument("--highcut", type=float, default=2.17)
    parser.add_argument("--bandpass-order", type=int, default=3)
    parser.add_argument("--sg-window-length", type=int, default=15)
    parser.add_argument("--sg-polyorder", type=int, default=3)

    args = parser.parse_args()

    try:
        session_labels, used_prompt = resolve_training_session_labels(args)
    except (RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc

    session_start: Optional[datetime] = None
    session_id: Optional[str] = None
    session_metadata: Optional[dict[str, object]] = None
    session_meta_path: Optional[str] = None

    output_dir = args.output_dir
    if session_labels is not None:
        session_start = datetime.now()
        session_id = build_session_id(session_labels, session_start)
        output_dir = build_subject_output_dir(args.output_dir, session_labels["subject"])
        session_metadata = build_session_metadata(
            session_labels, session_id, session_start, output_dir
        )

    ensure_dir(output_dir)

    if session_metadata is not None and session_id is not None:
        session_meta_path = os.path.join(
            output_dir, build_tagged_filename(session_id, "session_labels.json")
        )
        write_session_metadata(session_meta_path, session_metadata)
        print_session_label_summary(session_metadata)

        if used_prompt:
            try:
                input("标签确认完成，按回车开始采集...")
            except EOFError:
                pass

    csi_csv_path = os.path.join(
        output_dir, build_tagged_filename(session_id, "csi_data.csv")
    )
    csi_f, csi_writer = open_csv_with_header(csi_csv_path, CSI_DATA_HEADER)

    model = None
    if args.collect_training_data:
        print("Training data collection mode ON -> save raw CSI only")
    else:
        from tensorflow import keras

        model = keras.models.load_model(args.model_path, safe_mode=False)
        print(f"Inference mode ON -> loaded model: {args.model_path}")

    stats = Stats()
    shaped_window: deque[np.ndarray] = deque(maxlen=args.window_size)

    print(f"Opening serial port: {args.port} @ {args.baud}")
    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=args.timeout,
    )

    print("Start logging and processing...")
    print(f"output dir -> {output_dir}")
    print(f"csi       -> {csi_csv_path}")
    if session_meta_path is not None:
        print(f"labels    -> {session_meta_path}")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                stats.maybe_print(args.stats_interval)
                continue

            host_rx_us = time.time_ns() // 1000
            host_rx_iso = datetime.now().isoformat(timespec="milliseconds")

            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                stats.bad_lines += 1
                continue

            if not line:
                stats.maybe_print(args.stats_interval)
                continue

            stats.raw_lines += 1

            if not line.startswith("CSI_DATA,"):
                stats.maybe_print(args.stats_interval)
                continue

            try:
                row = parse_csi_data_line(line)
                csi_writer.writerow(row + [str(host_rx_us), host_rx_iso])
                csi_f.flush()
                stats.csi_data_lines += 1

                if not args.collect_training_data:
                    amplitudes = parse_csi_amplitudes(row[9])
                    dc_removed = remove_dc(amplitudes, fs=args.fs)
                    bandpassed = butter_bandpass_filter(
                        dc_removed,
                        args.lowcut,
                        args.highcut,
                        args.fs,
                        order=args.bandpass_order,
                    )
                    shaped = savitzky_golay_smooth(
                        bandpassed,
                        window_length=args.sg_window_length,
                        polyorder=args.sg_polyorder,
                    )

                    shaped_window.append(shaped.astype(np.float32))
                    if len(shaped_window) == args.window_size:
                        shaped_data_np = np.stack(shaped_window, axis=0)
                        shaped_data_np = shaped_data_np.reshape(
                            (1, shaped_data_np.shape[0], shaped_data_np.shape[1])
                        )
                        pred = model.predict(shaped_data_np, verbose=0)
                        stats.pred_count += 1
                        print(
                            f"PREDICT,pkt_count={row[1]},"
                            f"rx_ts={row[7]},pred={float(pred[0][0]):.6f}"
                        )

            except Exception as exc:
                stats.bad_csi_lines += 1
                print(f"[PARSE_ERROR] {type(exc).__name__}: {exc}")

            stats.maybe_print(args.stats_interval)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()
        csi_f.close()

        if (
            session_metadata is not None
            and session_meta_path is not None
            and session_start is not None
        ):
            session_end = datetime.now()
            session_metadata["session_end_time"] = session_end.isoformat(
                timespec="seconds"
            )
            session_metadata["duration_seconds"] = round(
                (session_end - session_start).total_seconds(), 3
            )
            session_metadata["stats"] = build_stats_summary(stats)
            session_metadata["files"] = {
                "csi_data_csv": os.path.abspath(csi_csv_path),
            }
            write_session_metadata(session_meta_path, session_metadata)


if __name__ == "__main__":
    main()