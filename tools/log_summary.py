#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List


def load_records(path: Path) -> List[Dict[str, Any]]:
    records: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                print(f"Skipping malformed JSON on line {line_number}.")
                continue
            if isinstance(value, dict):
                records.append(value)
    return records


def summarize(records: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    events = [record for record in records if record.get("record_type") == "event"]
    warnings = [event for event in events if event.get("severity") == "WARNING"]
    errors = [event for event in events if event.get("severity") == "ERROR"]
    timestamps = [event.get("timestamp") for event in errors if event.get("timestamp")]
    unique_error_types = sorted({str(event.get("event_type", "UNKNOWN")) for event in errors})
    return {
        "total_warnings": len(warnings),
        "total_errors": len(errors),
        "unique_error_types": unique_error_types,
        "first_error_timestamp": min(timestamps) if timestamps else "N/A",
        "last_error_timestamp": max(timestamps) if timestamps else "N/A",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize structured drone-system logs.")
    parser.add_argument("log_file", type=Path)
    args = parser.parse_args()

    summary = summarize(load_records(args.log_file))
    print(f"Total warnings: {summary['total_warnings']}")
    print(f"Total errors: {summary['total_errors']}")
    types = ", ".join(summary["unique_error_types"]) if summary["unique_error_types"] else "None"
    print(f"Unique error types: {types}")
    print(f"First error timestamp: {summary['first_error_timestamp']}")
    print(f"Last error timestamp: {summary['last_error_timestamp']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
