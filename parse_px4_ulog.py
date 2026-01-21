import argparse
from pathlib import Path
from typing import Iterable, Optional, Tuple

import tkinter as tk
from tkinter import filedialog

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pyulog import ULog


def quaternion_to_euler(q0: Iterable[float], q1: Iterable[float], q2: Iterable[float], q3: Iterable[float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    q0 = np.asarray(q0)
    q1 = np.asarray(q1)
    q2 = np.asarray(q2)
    q3 = np.asarray(q3)

    roll = np.arctan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
    pitch = np.arcsin(np.clip(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def extract_attitude(ulog: ULog) -> pd.DataFrame:
    dataset = next((d for d in ulog.data_list if d.name == "vehicle_attitude"), None)
    if dataset is None:
        raise RuntimeError("Topic 'vehicle_attitude' not found in log")

    data = dataset.data
    ts = np.asarray(data["timestamp"], dtype=np.float64)
    time_s = (ts - ts[0]) * 1e-6

    roll_deg, pitch_deg, yaw_deg = quaternion_to_euler(
        data["q[0]"], data["q[1]"], data["q[2]"], data["q[3]"]
    )

    return pd.DataFrame({
        "time_s": time_s,
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg,
    })


def plot_attitude(df: pd.DataFrame, output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(df["time_s"], df["roll_deg"], label="roll")
    ax.plot(df["time_s"], df["pitch_deg"], label="pitch")

    ax.set_title("PX4 Attitude Angles")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angle [deg]")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend()
    fig.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="从 PX4 ULog 绘制姿态角并保存图片")
    parser.add_argument("--log", type=Path, default=None, help="PX4 .ulg 日志文件路径；若未提供将弹出选择框")
    parser.add_argument("--out", type=Path, default=None, help="保存 PNG 的路径（默认：与日志同名文件夹/attitude_angles.png）")
    args = parser.parse_args()

    log_path: Optional[Path] = args.log
    if log_path is None:
        root = tk.Tk()
        root.withdraw()
        root.update()
        file_path = filedialog.askopenfilename(
            title="选择 PX4 ULog 文件",
            filetypes=[("ULog files", "*.ulg"), ("All files", "*.*")],
        )
        root.destroy()
        if not file_path:
            raise SystemExit("未选择日志文件，已退出。")
        log_path = Path(file_path)

    if not log_path.exists():
        raise SystemExit(f"日志文件不存在: {log_path}")

    ulog = ULog(str(log_path))

    out_path = args.out if args.out is not None else log_path.with_suffix("") / "attitude_angles.png"

    attitude_df = extract_attitude(ulog)
    plot_attitude(attitude_df, out_path)
    print(f"Attitude plot saved to: {out_path}")


if __name__ == "__main__":
    main()
