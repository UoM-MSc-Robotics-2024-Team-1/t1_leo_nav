from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    pkg_name = 't1_joy'

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0', 'coalesce_interval': 0.02, 'autorepeat_rate': 30.0}]
    )
    

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[
            os.path.join(get_package_share_directory(pkg_name), 'config/joy.yaml'),],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))]
    )

    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    
    return ld