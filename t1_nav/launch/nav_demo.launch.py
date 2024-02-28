from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os



def generate_launch_description():
    ld = LaunchDescription()

    # Parameters, Nodes and Launch files go here

    # Declare package directory
    pkg_nav = get_package_share_directory('t1_nav')

    # Necessary fixes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behaviour_server',
        'bt_navigator',
    ]

    # LOAD PARAMETERS FROM YAML FILES
    config_bt_nav = PathJoinSubstitution([pkg_nav, 'config', 'bt_nav.yaml'])
    config_planner = PathJoinSubstitution([pkg_nav, 'config', 'planner.yaml'])
    config_controller = PathJoinSubstitution([pkg_nav, 'config', 'controller.yaml'])

    # Include t1_rover to spawn the robot and launch gazebo and rviz
    launch_rover = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('t1_rover'), '/launch', '/spawn_robot.launch.py']),
    launch_arguments={}.items(),
    )

    # Include SLAM Toolbox standard launch file
    launch_slamtoolbox = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
    launch_arguments={}.items(),
    )

    # Behaviour Tree Navigator
    node_bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[config_bt_nav],
        remappings=remappings,
    )

    # Behaviour Tree Server
    node_behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behaviour_server',
        output='screen',
        parameters=[config_bt_nav],
        remappings=remappings,
    )

    # Planner Server Node
    node_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config_planner],
        remappings=remappings,
    )

    # Controller Server Node
    node_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[config_controller],
        remappings=remappings,
    )

    # Lifecycle Node Manager to automatically start lifecycles nodes (from list)
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )

     # Lifecycle Node Manager to automatically start lifecycles nodes (from list)
    node_explore = Node(
        package='t1_nav',
        executable='explore',
        name='explore',
        output='screen',
    )


    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_rover)
    ld.add_action(launch_slamtoolbox)
    ld.add_action(node_bt_nav)
    ld.add_action(node_behaviour)
    ld.add_action(node_planner)
    ld.add_action(node_controller)
    ld.add_action(node_lifecycle_manager)
    #ld.add_action(node_explore)

    return ld