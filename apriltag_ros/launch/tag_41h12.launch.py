#!/usr/bin/env python3
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

cfg_41h12 = {
    "image_transport": "raw",
    "family": "Standard41h12",
    "size": 0.1,
    "max_hamming": 0,# temp disabled
    "z_up": True,
    "tag_ids": [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28],
    "tag_frames": ["marker"],
    "tag_size": [0.11],
}

def generate_launch_description():
    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[("/image_rect", "/camera/image_raw"), ("/camera_info", "/camera/camera_info")],
        parameters=[cfg_41h12])
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])