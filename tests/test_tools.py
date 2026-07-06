from datetime import datetime, timedelta, timezone

from tools.ci_check import validate
from tools.log_summary import summarize


def test_log_summary_counts_required_fields():
    records = [
        {
            "record_type": "event",
            "severity": "WARNING",
            "event_type": "LOW_RTF",
            "timestamp": "2026-07-05T10:00:00.000Z",
        },
        {
            "record_type": "event",
            "severity": "ERROR",
            "event_type": "CAR_TIMEOUT",
            "timestamp": "2026-07-05T10:00:01.000Z",
        },
        {
            "record_type": "event",
            "severity": "ERROR",
            "event_type": "CAR_TIMEOUT",
            "timestamp": "2026-07-05T10:00:03.000Z",
        },
    ]
    summary = summarize(records)
    assert summary["total_warnings"] == 1
    assert summary["total_errors"] == 2
    assert summary["unique_error_types"] == ["CAR_TIMEOUT"]
    assert summary["first_error_timestamp"] == "2026-07-05T10:00:01.000Z"
    assert summary["last_error_timestamp"] == "2026-07-05T10:00:03.000Z"


def make_telemetry(duration_s: int, altitude: float = 20.0):
    start = datetime(2026, 7, 5, 10, 0, tzinfo=timezone.utc)
    return [
        {
            "record_type": "telemetry",
            "timestamp": (start + timedelta(seconds=index)).isoformat().replace("+00:00", "Z"),
            "drone_z": altitude,
            "car_position_rate_hz": 50.0,
        }
        for index in range(duration_s + 1)
    ]


def test_ci_validation_passes_a_complete_healthy_run():
    failures = validate(
        make_telemetry(60),
        final_window_s=30.0,
        minimum_altitude_m=1.0,
        minimum_samples=20,
        minimum_run_s=50.0,
        minimum_car_rate_hz=1.0,
    )
    assert failures == []


def test_ci_validation_rejects_a_short_false_positive():
    failures = validate(
        make_telemetry(10),
        final_window_s=30.0,
        minimum_altitude_m=1.0,
        minimum_samples=5,
        minimum_run_s=50.0,
        minimum_car_rate_hz=1.0,
    )
    assert any("covered only" in failure for failure in failures)


def test_ci_validation_rejects_a_dead_car_stream():
    records = make_telemetry(60)
    for record in records[-31:]:
        record["car_position_rate_hz"] = 0.0
    failures = validate(
        records,
        final_window_s=30.0,
        minimum_altitude_m=1.0,
        minimum_samples=20,
        minimum_run_s=50.0,
        minimum_car_rate_hz=1.0,
    )
    assert any("car-position rate" in failure for failure in failures)


def test_structured_log_schema_and_iso_timestamp(tmp_path):
    import json

    from drone_system.structured_logging import StructuredLog

    path = tmp_path / "events.jsonl"
    logger = StructuredLog("test_component", str(path))
    logger.event("ERROR", "TEST_FAILURE", "A plain-English failure description.")

    record = json.loads(path.read_text(encoding="utf-8").strip())
    assert record["record_type"] == "event"
    assert record["severity"] == "ERROR"
    assert record["component"] == "test_component"
    assert record["event_type"] == "TEST_FAILURE"
    assert record["description"] == "A plain-English failure description."
    assert record["timestamp"].endswith("Z")
    assert "T" in record["timestamp"]
