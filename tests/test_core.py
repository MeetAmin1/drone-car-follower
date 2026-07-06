import math

import pytest

from drone_system.core import (
    ArmRetryPolicy,
    FollowerPlanner,
    Point3,
    PositionValidator,
    RtfEstimator,
    StaleDetector,
    enu_to_ned,
    ned_to_enu,
)


def test_position_jump_is_discarded_and_last_valid_is_held():
    validator = PositionValidator(max_jump_m=5.0)
    first = Point3(1.0, 2.0, 0.5)
    assert validator.update(first).accepted
    decision = validator.update(Point3(10.0, 2.0, 0.5))
    assert not decision.accepted
    assert decision.reason == "jump"
    assert decision.point == first
    assert decision.jump_m == pytest.approx(9.0)


def test_follower_waypoint_has_fixed_offset():
    planner = FollowerPlanner(offset_m=8.0, altitude_m=20.0, min_heading_speed_mps=0.1)
    previous = Point3(0.0, 0.0, 0.5)
    current = Point3(1.0, 0.0, 0.5)
    waypoint = planner.waypoint(current, previous, 0.1)
    assert math.hypot(current.x - waypoint.x, current.y - waypoint.y) == pytest.approx(8.0)
    assert waypoint.z == pytest.approx(20.0)


def test_stale_detector_uses_strict_greater_than_timeout():
    detector = StaleDetector(timeout_s=0.2)
    detector.mark(10.0)
    assert not detector.is_stale(10.2)
    assert detector.is_stale(10.200001)


def test_arm_policy_allows_initial_plus_three_retries():
    policy = ArmRetryPolicy(retry_count=3, retry_interval_s=2.0)
    now = 0.0
    for expected in range(1, 5):
        assert policy.should_send(now)
        assert policy.record_command(now) == expected
        now += 2.0
    assert not policy.should_send(now)
    assert policy.exhausted_after_wait(now)


def test_rtf_estimator_detects_half_speed():
    estimator = RtfEstimator(window_s=2.0)
    estimator.add(0.0, 0.0)
    estimator.add(1.0, 0.5)
    assert estimator.estimate(1.0) == pytest.approx(0.5)


def test_enu_ned_round_trip():
    original = Point3(3.0, -4.0, 20.0)
    assert ned_to_enu(enu_to_ned(original)) == original
