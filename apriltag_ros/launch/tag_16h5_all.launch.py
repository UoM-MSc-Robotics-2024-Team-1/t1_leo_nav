#!/usr/bin/env python3
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# detect only 16h5 tags with id=0
cfg_16h5 = {
    "image_transport": "raw",
    "family": "Standard41h12",
    "size": 0.1,
    "max_hamming": 0,# temp disabled
    "z_up": True,
    "tag_ids": [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28],
    #"tag_ids": [10],# temp
    #"tag_frames": ["marker"],# temp
    "tag_frames": ["marker0","marker1","marker2","marker3","marker4","marker5","marker6","marker7","marker8","marker9","marker10","marker11","marker12","marker13","marker14","marker15","marker16","marker17","marker18","marker19","marker20","marker21","marker22","marker23","marker24","marker25","marker26","marker27","marker28"],
    "tag_size": [0.1],
}

def generate_launch_description():
    composable_node = ComposableNode(
        node_name='apriltag',
        package='apriltag_ros', node_plugin='AprilTagNode',
        #remappings=[("/image_rect/compressed", "/image_raw/compressed"), ("/camera_info", "/camera_info")], # recognized apriltag but had trouble with quaternions with webcam
        remappings=[("/image_rect", "/camera/image_raw"), ("/camera_info", "/camera/camera_info")],
        #remappings=[("/apriltag/image", "/camera/image_raw"), ("/apriltag/camera_info", "/camera/camera_info")],# old 1x
        parameters=[cfg_16h5])
    container = ComposableNodeContainer(
        node_name='tag_container',
        node_namespace='apriltag',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])