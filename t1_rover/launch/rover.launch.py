import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    ld = LaunchDescription()
    pkg_name = 't1_rover'

    # Declare the argument for use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Whether to run in simulation mode or not'
    )

    # Use xacro to process the files for simulation
    xacro_file_sim = os.path.join(get_package_share_directory('leo_description'), 'urdf/leo_sim.urdf.xacro')
    robot_description_raw_sim = xacro.process_file(xacro_file_sim).toxml()


    # Include the t1_sim launch file
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('t1_sim'), 'launch/sim_bringup.launch.py')])
    )

    # Launch lidar package
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('rplidar_ros'), 'launch/rplidar_a2m12_launch.py')])
    )

    # Launch robot state publisher node with simulation (URDF file)
    robot_state_publisher_sim = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_sim",
        output="both",
        parameters=[{"use_sim_time": True, "robot_description": robot_description_raw_sim}],
    )

    # Launch rviz2 with the nav2 configuration
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(pkg_name), 'rviz', 'nav2.rviz')],
    )

    # Filter imu data
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory(pkg_name), 'config/imu_filter_params.yaml')
        ],
    )

    # Filter odometry data, uses imu data
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory(pkg_name), 'config/localization_params.yaml'), 
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    # Dynamically add actions based on use_sim_time
    def conditionally_launch_nodes(context):
        if context.launch_configurations['use_sim_time'] == 'true':
            return [sim_launch, robot_state_publisher_sim]
        else:
            return [rplidar_launch, imu_filter_node, robot_localization_node] 

    # Add actions to LaunchDescription
    ld.add_action(use_sim_time_arg)
    ld.add_action(node_rviz)
    #ld.add_action(imu_filter_node)
    #ld.add_action(robot_localization_node)
    ld.add_action(OpaqueFunction(function=conditionally_launch_nodes))

    return ld
