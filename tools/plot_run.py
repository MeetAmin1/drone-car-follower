#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
from typing import Any, Dict, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_timestamp(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def load_telemetry(path: Path) -> List[Dict[str, Any]]:
    records: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if record.get("record_type") == "telemetry" and record.get("timestamp"):
                records.append(record)
    if not records:
        raise RuntimeError("No telemetry records were found in the log file.")
    records.sort(key=lambda item: item["timestamp"])
    return records


def values(records: List[Dict[str, Any]], key: str):
    return [record.get(key) for record in records]


def valid_pairs(xs, ys):
    pairs = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]
    if not pairs:
        return [], []
    return [p[0] for p in pairs], [p[1] for p in pairs]


def save_path_plot(records: List[Dict[str, Any]], out_dir: Path) -> None:
    car_x, car_y = valid_pairs(values(records, "car_x"), values(records, "car_y"))
    drone_x, drone_y = valid_pairs(values(records, "drone_x"), values(records, "drone_y"))
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(car_x, car_y, label="Car")
    ax.plot(drone_x, drone_y, label="Drone")
    ax.set_title("Drone XY path vs. car XY path")
    ax.set_xlabel("East / X (m)")
    ax.set_ylabel("North / Y (m)")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "xy_paths.png", dpi=180)
    plt.close(fig)


def save_time_plot(records, times, key, title, ylabel, filename, out_dir):
    y = values(records, key)
    pairs = [(t, value) for t, value in zip(times, y) if value is not None]
    fig, ax = plt.subplots(figsize=(9, 4.5))
    if pairs:
        ax.plot([p[0] for p in pairs], [p[1] for p in pairs])
    ax.set_title(title)
    ax.set_xlabel("Time from first record (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(out_dir / filename, dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate assessment plots from a JSONL run log.")
    parser.add_argument("log_file", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("plots"))
    args = parser.parse_args()

    records = load_telemetry(args.log_file)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    start = parse_timestamp(records[0]["timestamp"])
    times = [(parse_timestamp(record["timestamp"]) - start).total_seconds() for record in records]

    save_path_plot(records, args.out_dir)
    save_time_plot(
        records, times, "car_position_rate_hz", "Car position message arrival rate", "Messages / s",
        "message_arrival_rate.png", args.out_dir)
    save_time_plot(
        records, times, "gazebo_rtf", "Gazebo real-time factor", "RTF",
        "gazebo_rtf.png", args.out_dir)
    save_time_plot(
        records, times, "drone_z", "Drone altitude", "Altitude (m)",
        "drone_altitude.png", args.out_dir)

    print(f"Wrote plots to {args.out_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
