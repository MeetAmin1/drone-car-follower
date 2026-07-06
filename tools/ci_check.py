#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime, timedelta
import json
from pathlib import Path
from typing import Any, Dict, List


def parse_timestamp(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def load(path: Path) -> List[Dict[str, Any]]:
    records = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(record, dict) and record.get("timestamp"):
                records.append(record)
    return records


def validate(
    records: List[Dict[str, Any]],
    final_window_s: float,
    minimum_altitude_m: float,
    minimum_samples: int,
    minimum_run_s: float,
    minimum_car_rate_hz: float,
) -> List[str]:
    telemetry = [record for record in records if record.get("record_type") == "telemetry"]
    if not telemetry:
        return ["no telemetry records found"]

    telemetry.sort(key=lambda record: parse_timestamp(record["timestamp"]))
    first_time = parse_timestamp(telemetry[0]["timestamp"])
    end = parse_timestamp(telemetry[-1]["timestamp"])
    start = end - timedelta(seconds=final_window_s)

    failures: List[str] = []
    run_span_s = (end - first_time).total_seconds()
    if run_span_s < minimum_run_s:
        failures.append(
            f"telemetry covered only {run_span_s:.3f} s, below the required {minimum_run_s:.3f} s"
        )

    final_telemetry = [
        record
        for record in telemetry
        if parse_timestamp(record["timestamp"]) >= start and record.get("drone_z") is not None
    ]
    final_errors = [
        record
        for record in records
        if record.get("record_type") == "event"
        and record.get("severity") == "ERROR"
        and parse_timestamp(record["timestamp"]) >= start
    ]

    if len(final_telemetry) < minimum_samples:
        failures.append(f"only {len(final_telemetry)} altitude samples in the final window")
    if final_telemetry:
        minimum = min(float(record["drone_z"]) for record in final_telemetry)
        if minimum <= minimum_altitude_m:
            failures.append(
                f"minimum final-window altitude was {minimum:.3f} m, not above {minimum_altitude_m:.3f} m"
            )
    final_car_rates = [
        float(record["car_position_rate_hz"])
        for record in final_telemetry
        if record.get("car_position_rate_hz") is not None
    ]
    if not final_car_rates:
        failures.append("no car-position arrival-rate samples in the final window")
    elif min(final_car_rates) < minimum_car_rate_hz:
        failures.append(
            f"minimum final-window car-position rate was {min(final_car_rates):.3f} Hz, "
            f"below {minimum_car_rate_hz:.3f} Hz"
        )

    if final_errors:
        types = sorted({str(record.get("event_type", "UNKNOWN")) for record in final_errors})
        failures.append(f"errors in final window: {', '.join(types)}")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate the final integration-test window.")
    parser.add_argument("log_file", type=Path)
    parser.add_argument("--final-window-s", type=float, default=30.0)
    parser.add_argument("--minimum-altitude-m", type=float, default=1.0)
    parser.add_argument("--minimum-samples", type=int, default=20)
    parser.add_argument("--minimum-run-s", type=float, default=50.0)
    parser.add_argument("--minimum-car-rate-hz", type=float, default=1.0)
    args = parser.parse_args()

    failures = validate(
        load(args.log_file),
        final_window_s=args.final_window_s,
        minimum_altitude_m=args.minimum_altitude_m,
        minimum_samples=args.minimum_samples,
        minimum_run_s=args.minimum_run_s,
        minimum_car_rate_hz=args.minimum_car_rate_hz,
    )
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1

    print(
        f"PASS: the run covered at least {args.minimum_run_s:.0f} s, final-window altitude stayed above "
        f"{args.minimum_altitude_m:.1f} m, car updates remained active, and no errors occurred in "
        f"the final {args.final_window_s:.0f} s."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
