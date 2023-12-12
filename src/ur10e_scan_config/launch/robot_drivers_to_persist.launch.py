# -*- coding: utf-8 -*-
# Copyright 2023 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from moveit_studio_utils_py.launch_common import empty_gen, get_launch_file
from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
)


def generate_launch_description():
    system_config_parser = SystemConfigParser()
    hardware_config = system_config_parser.get_hardware_config()
    controller_config = system_config_parser.get_ros2_control_config()
    cameras_config = system_config_parser.get_cameras_config()

    included_launch_files = []
    camera_frames = []

    dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": hardware_config["ip"]}],
    )

    protective_stop_manager_node = Node(
        package="moveit_studio_ur_pstop_manager",
        executable="protective_stop_manager_node",
        name="protective_stop_manager_node",
        output="screen",
        parameters=[
            {
                "controllers_default_active": controller_config.get(
                    "controllers_active_at_startup", empty_gen()
                ),
                "controllers_default_not_active": controller_config.get(
                    "controllers_inactive_at_startup", empty_gen()
                ),
            }
        ],
    )

    nodes_to_launch = [
        dashboard_client_node,
        protective_stop_manager_node,
    ]

    for camera in cameras_config:
        camera = camera[next(iter(camera))]
        camera_frames.append(camera["camera_name"] + "_color_frame")

        # The Realsense Node wants the a string instead of bool for "enable_pointcloud"
        enable_pointcloud = "false"
        if camera["enable_pointcloud"]:
            enable_pointcloud = "true"

        included_launch_files.append(
            IncludeLaunchDescription(
                get_launch_file("realsense2_camera", "launch/rs_launch.py"),
                launch_arguments={
                    "serial_no": "'" + str(camera["serial_no"]) + "'",
                    "pointcloud.enable": enable_pointcloud,
                    "device_type": camera["device_type"],
                    "camera_name": camera["camera_name"],
                    "base_frame_id": camera["camera_name"] + "_link",
                    # All 3 params (fps/width/height) must be set
                    "depth_module.profile": f"{camera['image_width']}x{camera['image_height']}x{camera['framerate']}",
                    "rgb_camera.profile": f"{camera['image_width']}x{camera['image_height']}x{camera['framerate']}",
                    "pointcloud.ordered_pc": "true",
                    "decimation_filter.enable": "false",
                }.items(),
            )
        )

    frame_pair_params = [
        {
            "world_frame": "world",
            "camera_frames": camera_frames,
        }
    ]

    camera_transforms_node = Node(
        package="moveit_studio_agent",
        executable="camera_transforms_node",
        name="camera_transforms_node",
        output="screen",
        parameters=frame_pair_params,
    )

    nodes_to_launch.append(camera_transforms_node)

    return LaunchDescription(included_launch_files + nodes_to_launch)
