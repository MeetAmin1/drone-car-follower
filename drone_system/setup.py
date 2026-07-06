from glob import glob
import os
from setuptools import find_packages, setup

package_name = "drone_system"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Meet Amin",
    maintainer_email="meetamin2003@gmail.com",
    description="PX4 SITL drone follows a scripted Gazebo car with hardened ROS 2 failure handling.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "car_position_node = drone_system.car_position_node:main",
            "follower_node = drone_system.follower_node:main",
            "gazebo_monitor_node = drone_system.gazebo_monitor_node:main",
            "gcs_heartbeat_node = drone_system.gcs_heartbeat_node:main",
            "px4_offboard_node = drone_system.px4_offboard_node:main",
            "telemetry_logger_node = drone_system.telemetry_logger_node:main",
        ],
    },
)
