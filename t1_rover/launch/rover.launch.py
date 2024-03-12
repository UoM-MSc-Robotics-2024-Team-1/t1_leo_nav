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
        'use_sim_time', default_value='true', description='Whether to run in simulation mode'
    )

    # Use xacro to process the files for SIM and not SIM
    xacro_file_sim = os.path.join(get_package_share_directory(pkg_name), 'urdf/leo_sim.urdf.xacro')
    robot_description_raw_sim = xacro.process_file(xacro_file_sim).toxml()

    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf/leo.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    ''' Pass through the package path to directory
    pkg_share_directory = get_package_share_directory(pkg_name)

    # Set an environment variable to use in the URDF/Xacro file
    set_pkg_dir_env = SetEnvironmentVariable(
        'PKG_DIR', pkg_share_directory
    )

    urdf_file_path = os.path.join(pkg_share_directory, 'urdf', 'robot.urdf.xacro')

    robot_description = Command(['xacro ', urdf_file_path, ' pkg_dir:=', pkg_share_directory])
    '''

    # Include the t1_sim launch file
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('t1_sim'), 'launch/sim_bringup.launch.py')])
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('rplidar_ros'), 'launch/rplidar_a2m12_launch.py')])
    )

    # Launch robot state publisher node with simulation
    robot_state_publisher_sim = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_sim",
        output="both",
        parameters=[{"use_sim_time": True, "robot_description": robot_description_raw_sim}],
    )

    # Launch robot state publisher node without simulation
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": False, "robot_description": robot_description_raw}],
    )

    # Rviz node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(pkg_name), 'rviz', 'nav2.rviz')],
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory(pkg_name), 'config/localization_params.yaml'), 
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/firmware/wheel_odom', '/odom'),
        ]
    )

    # RPLIDAR node definition corrected (removed trailing comma)
    node_rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # Adjust baudrate as needed
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }]
    )

    # Dynamically add actions based on use_sim_time
    def conditionally_launch_nodes(context):
        if context.launch_configurations['use_sim_time'] == 'true':
            return [sim_launch, robot_state_publisher_sim]
        else:
            return [robot_state_publisher, rplidar_launch]

    # Add actions to LaunchDescription
    ld.add_action(use_sim_time_arg)
    ld.add_action(node_rviz)
    ld.add_action(robot_localization_node)
    ld.add_action(OpaqueFunction(function=conditionally_launch_nodes))

    return ld
