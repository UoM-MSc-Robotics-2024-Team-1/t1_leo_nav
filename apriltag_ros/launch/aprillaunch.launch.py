#!/usr/bin/env python3

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# detect only 16h5 tags with id=0 from compressed image stream (ensure image transport plugins and image common exist/ build from source)
cfg_16h5 = {
    #required params
    "image_transport": "compressed",
    "family": "16h5",
    "size": 0.162,
    "z_up": True,

    #optional detection tuning params
    #"max_hamming": 0,
    #"decimate": 1.0,
    #"blur": 0.0,
    #"refine-edges": 1,
    #"threads": 1,
    #"debug": 0,

    #optional customization of tag list/display
    "tag_ids": [0],
    "tag_frames": ["marker"],
    "tag_sizes": [0.162],
}

def generate_launch_description():
    composable_node = ComposableNode(
        name='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[("/image_rect/compressed", "/camera/image_raw/compressed"), ("/camera_info", "/camera_info")],
        parameters=[cfg_16h5])
    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return launch.LaunchDescription([container])