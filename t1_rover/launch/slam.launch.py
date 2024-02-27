from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()
    


    # Add launch file from sim_robot.launch.py
    include_sim_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('t1_rover'), '/launch', '/spawn_robot.launch.py']),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )


    # Add ros2 launch slam_toolbox online_async_launch.py
    include_slam_toolbox_online_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'scan_topic': '/scan',
            'odom_topic': '/odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'global_frame': 'map',
            }.items()
    )

    # Add map_saver node
    node_map_saver = Node(
        package='map_server',
        executable='map_saver',
        name='map_saver',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('t1_rover'), 'maps', 'my_map.yaml')]
    )



    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(include_sim_robot_launch)
    ld.add_action(include_slam_toolbox_online_async_launch)
    ld.add_action(node_map_saver)
    return ld
