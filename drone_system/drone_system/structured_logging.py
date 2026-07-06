"""Atomic JSON-lines logging shared by all nodes."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import time
from typing import Any, Dict, Optional


_WRITE_LOCK = threading.Lock()


def iso_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def resolve_log_path(configured_path: str = "") -> Path:
    candidate = os.environ.get("DRONE_SYSTEM_LOG", "").strip() or configured_path.strip()
    if not candidate:
        candidate = "logs/run.jsonl"
    return Path(candidate).expanduser().resolve()


class StructuredLog:
    def __init__(self, component: str, configured_path: str = "", ros_logger: Any = None) -> None:
        self.component = component
        self.path = resolve_log_path(configured_path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.ros_logger = ros_logger
        self.started_monotonic = time.monotonic()

    def event(
        self,
        severity: str,
        event_type: str,
        description: str,
        **fields: Any,
    ) -> None:
        severity_upper = severity.upper()
        record: Dict[str, Any] = {
            "record_type": "event",
            "timestamp": iso_timestamp(),
            "elapsed_s": round(time.monotonic() - self.started_monotonic, 6),
            "severity": severity_upper,
            "component": self.component,
            "event_type": event_type,
            "description": description,
        }
        record.update(fields)
        self._write(record)

        if self.ros_logger is not None:
            message = f"[{self.component}] {description}"
            if severity_upper == "ERROR":
                self.ros_logger.error(message)
            elif severity_upper == "WARNING":
                self.ros_logger.warning(message)
            else:
                self.ros_logger.info(message)

    def telemetry(self, **fields: Any) -> None:
        record: Dict[str, Any] = {
            "record_type": "telemetry",
            "timestamp": iso_timestamp(),
            "elapsed_s": round(time.monotonic() - self.started_monotonic, 6),
            "component": self.component,
        }
        record.update(fields)
        self._write(record)

    def _write(self, record: Dict[str, Any]) -> None:
        payload = (json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
        with _WRITE_LOCK:
            fd = os.open(self.path, os.O_APPEND | os.O_CREAT | os.O_WRONLY, 0o644)
            try:
                os.write(fd, payload)
                # Preserve safety events without forcing a disk flush for every 10 Hz telemetry line.
                if record.get("record_type") == "event" and record.get("severity") == "ERROR":
                    os.fsync(fd)
            finally:
                os.close(fd)
