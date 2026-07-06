from pathlib import Path
import ast
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
PARAMS = ROOT / "drone_system" / "config" / "params.yaml"


def test_two_ros_packages_are_discoverable() -> None:
    packages = sorted(path.relative_to(ROOT).as_posix() for path in ROOT.glob("*/package.xml"))
    assert packages == ["car_motion_plugin/package.xml", "drone_system/package.xml"]
    for package in packages:
        ET.parse(ROOT / package)


def test_required_thresholds_live_in_params_yaml() -> None:
    document = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    follower = document["follower_node"]["ros__parameters"]
    controller = document["px4_offboard_node"]["ros__parameters"]
    monitor = document["gazebo_monitor_node"]["ros__parameters"]

    assert follower["car_timeout_s"] == 0.2
    assert follower["max_position_jump_m"] == 5.0
    assert controller["arm_retry_count"] == 3
    assert monitor["rtf_min"] == 0.8
    assert monitor["rtf_warning_interval_s"] == 5.0


def test_launch_and_workflow_contracts() -> None:
    launch_path = ROOT / "drone_system" / "launch" / "full_stack.launch.py"
    ast.parse(launch_path.read_text(encoding="utf-8"))
    workflow = (ROOT / ".github" / "workflows" / "integration_test.yml").read_text(encoding="utf-8")
    assert "run_integration.sh 60" in workflow
    assert "actions/upload-artifact" in workflow
    launch_source = launch_path.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("headless", default_value="true")' in launch_source
    assert '"/car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"' in launch_source
    assert "dynamic_pose/info" not in launch_source
    assert 'env["GZ_SIM_SERVER_CONFIG_PATH"]' in launch_source
    assert 'str(px4_gz_plugins)' in launch_source
    assert 'PX4_SYS_AUTOSTART=4001' in launch_source
    assert 'PX4_SIM_MODEL=gz_x500' in launch_source
    assert 'make px4_sitl gz_x500' not in launch_source


def test_follower_is_topic_isolated_from_gazebo() -> None:
    source = (ROOT / "drone_system" / "drone_system" / "follower_node.py").read_text(encoding="utf-8")
    assert '"/car/position"' in source
    assert '"/drone/waypoint"' in source
    assert "ros_gz" not in source
    assert "gazebo" not in source.lower()


def test_car_position_comes_from_dedicated_odometry_stream() -> None:
    world = (ROOT / "drone_system" / "worlds" / "car_follow.sdf").read_text(encoding="utf-8")
    relay = (ROOT / "drone_system" / "drone_system" / "car_position_node.py").read_text(encoding="utf-8")
    assert "gz-sim-odometry-publisher-system" in world
    assert "<odom_topic>/car/odometry</odom_topic>" in world
    assert "from nav_msgs.msg import Odometry" in relay
    assert "SetWorldPoseCmd" not in relay


def test_traceability_covers_every_named_deliverable() -> None:
    traceability = (ROOT / "REQUIREMENTS_TRACEABILITY.md").read_text(encoding="utf-8")
    for required in (
        "/car/position",
        "/drone/waypoint",
        "CAR_POSITION_TIMEOUT",
        "PX4_ARM_FAILED",
        "CAR_POSITION_JUMP",
        "GAZEBO_RTF_LOW",
        "tools/log_summary.py",
        "tools/plot_run.py",
        ".github/workflows/integration_test.yml",
        "ANALYSIS.md",
        "EMAIL_SUBMISSION.txt",
    ):
        assert required in traceability


def test_px4_117_interface_and_preflight_contract() -> None:
    source = (ROOT / "drone_system" / "drone_system" / "px4_offboard_node.py").read_text(encoding="utf-8")
    assert '"/fmu/out/vehicle_status_v1"' in source
    assert '"/fmu/out/vehicle_local_position_v1"' in source
    assert "pre_flight_checks_pass" in source
    assert "px4_preflight_timeout_s" in source


def test_dependency_versions_are_pinned() -> None:
    dockerfile = (ROOT / "Dockerfile").read_text(encoding="utf-8")
    assert "ARG PX4_VERSION=v1.17.0" in dockerfile
    assert "--branch v1.17.0 https://github.com/PX4/px4_msgs.git" in dockerfile
