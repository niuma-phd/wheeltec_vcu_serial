#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fail-closed offline launch for the Wheeltec ROS 2 adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("wheeltec_vcu_serial_ros2"))
    offline_config = str(package_share / "config" / "offline.ini")

    config_file = LaunchConfiguration("config_file")
    acknowledge = LaunchConfiguration("acknowledge_unverified_protocol")
    enable_actuation = LaunchConfiguration("enable_actuation")
    confirmation = LaunchConfiguration("operator_confirmation")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=offline_config),
            DeclareLaunchArgument(
                "acknowledge_unverified_protocol", default_value="false"
            ),
            DeclareLaunchArgument("enable_actuation", default_value="false"),
            DeclareLaunchArgument("operator_confirmation", default_value=""),
            Node(
                package="wheeltec_vcu_serial_ros2",
                executable="wheeltec_vcu_serial_adapter",
                name="wheeltec_vcu_serial_adapter",
                output="screen",
                parameters=[
                    {
                        "config_file": config_file,
                        "acknowledge_unverified_protocol": acknowledge,
                        "enable_actuation": enable_actuation,
                        "operator_confirmation": confirmation,
                    }
                ],
            ),
        ]
    )
