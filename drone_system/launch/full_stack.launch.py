from __future__ import annotations

import os
import shlex

import yaml
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def _launch_setup(context):
    share = Path(get_package_share_directory("drone_system"))
    plugin_prefix = Path(get_package_prefix("car_motion_plugin"))
    params_file = share / "config" / "params.yaml"
    world_file = share / "worlds" / "car_follow.sdf"

    with params_file.open("r", encoding="utf-8") as handle:
        parameter_document = yaml.safe_load(handle)
    launch_parameters = parameter_document["full_stack_launch"]["ros__parameters"]
    gazebo_startup_timeout_s = float(launch_parameters["gazebo_startup_timeout_s"])
    gazebo_startup_poll_s = float(launch_parameters["gazebo_startup_poll_s"])

    px4_dir = Path(LaunchConfiguration("px4_dir").perform(context)).expanduser().resolve()
    log_file = Path(LaunchConfiguration("log_file").perform(context)).expanduser().resolve()
    headless = LaunchConfiguration("headless").perform(context).lower() in {"1", "true", "yes", "on"}

    px4_build = px4_dir / "build" / "px4_sitl_default"
    px4_binary = px4_build / "bin" / "px4"
    px4_gz_env = px4_build / "rootfs" / "gz_env.sh"
    px4_gz_plugins = px4_build / "src" / "modules" / "simulation" / "gz_plugins"
    px4_server_config = px4_dir / "src" / "modules" / "simulation" / "gz_bridge" / "server.config"
    required_paths = (px4_binary, px4_gz_env, px4_gz_plugins, px4_server_config)
    missing = [str(path) for path in required_paths if not path.exists()]
    if missing:
        raise RuntimeError("PX4 Gazebo runtime is incomplete; missing: " + ", ".join(missing))

    log_file.parent.mkdir(parents=True, exist_ok=True)
    log_file.write_text("", encoding="utf-8")

    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_paths = [
        str(share / "worlds"),
        str(px4_dir / "Tools" / "simulation" / "gz" / "models"),
        str(px4_dir / "Tools" / "simulation" / "gz" / "worlds"),
        str(Path.home() / ".simulation-gazebo" / "models"),
        str(Path.home() / ".simulation-gazebo" / "worlds"),
    ]
    if existing_resource_path:
        resource_paths.append(existing_resource_path)

    env = os.environ.copy()
    env["DRONE_SYSTEM_LOG"] = str(log_file)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(
        [
            str(plugin_prefix / "lib"),
            str(px4_gz_plugins),
            env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
        ]
    ).strip(os.pathsep)
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(resource_paths)
    # PX4's server configuration loads the IMU, barometer, magnetometer,
    # NavSat, contact and rendering systems required by the x500 model.
    env["GZ_SIM_SERVER_CONFIG_PATH"] = str(px4_server_config)
    env["GZ_TRANSPORT_LOCALHOST_ONLY"] = "1"
    env["GZ_IP"] = "127.0.0.1"
    env["PX4_GZ_STANDALONE"] = "1"
    env["PX4_GZ_WORLD"] = "default"

    gazebo_cmd = ["gz", "sim", "-r"]
    if headless:
        gazebo_cmd.append("-s")
    gazebo_cmd.append(str(world_file))

    gazebo = ExecuteProcess(
        cmd=gazebo_cmd,
        name="gazebo",
        output="screen",
        additional_env=env,
        sigterm_timeout="10",
        sigkill_timeout="5",
    )

    timeout_literal = shlex.quote(f"{gazebo_startup_timeout_s}s")
    poll_literal = shlex.quote(str(gazebo_startup_poll_s))
    px4_dir_literal = shlex.quote(str(px4_dir))
    px4_binary_literal = shlex.quote(str(px4_binary))
    px4_gz_env_literal = shlex.quote(str(px4_gz_env))
    wait_for_gazebo = (
        "until gz service -l 2>/dev/null | grep -q '^/world/default/create$'; "
        f"do sleep {poll_literal}; done"
    )
    # Execute the SITL binary directly instead of wrapping it in make. This
    # lets ROS launch signal the real PX4 process, preventing stale instances.
    px4_shell = (
        "set -e; "
        f"if ! timeout --foreground {timeout_literal} bash -lc {shlex.quote(wait_for_gazebo)}; then "
        "  echo 'Gazebo startup timeout'; exit 3; "
        "fi; "
        f"cd {px4_dir_literal}; "
        f"source {px4_gz_env_literal}; "
        "exec env PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 "
        "PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=default "
        f"{px4_binary_literal}"
    )

    px4 = ExecuteProcess(
        cmd=["bash", "-lc", px4_shell],
        name="px4_sitl",
        output="screen",
        additional_env=env,
        sigterm_timeout="10",
        sigkill_timeout="5",
    )

    agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        name="micro_xrce_agent",
        output="screen",
        additional_env=env,
        sigterm_timeout="5",
        sigkill_timeout="3",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        output="screen",
        additional_env=env,
    )

    common = {
        "parameters": [str(params_file), {"log_file": str(log_file)}],
        "output": "screen",
        "additional_env": env,
    }

    car_position = Node(package="drone_system", executable="car_position_node", name="car_position_node", **common)
    follower = Node(package="drone_system", executable="follower_node", name="follower_node", **common)
    monitor = Node(package="drone_system", executable="gazebo_monitor_node", name="gazebo_monitor_node", **common)
    heartbeat = Node(package="drone_system", executable="gcs_heartbeat_node", name="gcs_heartbeat_node", **common)
    telemetry = Node(package="drone_system", executable="telemetry_logger_node", name="telemetry_logger_node", **common)
    controller = Node(package="drone_system", executable="px4_offboard_node", name="px4_offboard_node", **common)

    critical_actions = [
        (gazebo, "Gazebo exited"),
        (px4, "PX4 SITL exited"),
        (agent, "Micro XRCE-DDS Agent exited"),
        (bridge, "ROS-Gazebo bridge exited"),
        (car_position, "Car position relay exited"),
        (follower, "Follower node exited"),
        (monitor, "Gazebo monitor exited"),
        (heartbeat, "GCS heartbeat node exited"),
        (telemetry, "Telemetry logger exited"),
        (controller, "PX4 controller exited"),
    ]
    shutdown_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=action,
                on_exit=[EmitEvent(event=Shutdown(reason=reason))],
            )
        )
        for action, reason in critical_actions
    ]

    # Register handlers before starting processes so an immediate startup failure
    # cannot escape launch-level shutdown handling.
    return shutdown_handlers + [action for action, _ in critical_actions]



def generate_launch_description():
    default_px4 = os.environ.get("PX4_AUTOPILOT_DIR", str(Path.home() / "PX4-Autopilot"))
    default_log = os.environ.get("DRONE_SYSTEM_LOG", str(Path.cwd() / "logs" / "run.jsonl"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("px4_dir", default_value=default_px4),
            DeclareLaunchArgument("log_file", default_value=default_log),
            DeclareLaunchArgument("headless", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
