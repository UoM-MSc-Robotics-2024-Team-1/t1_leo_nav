import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter, Node
import xacro
from launch import LaunchDescription

def generate_launch_description():

    ld = LaunchDescription()

    pkg_name = 't1_rover'

    # Declare the argument for use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Whether to run in simulation mode'
    )

    # Use xacro to process the files
    #SIM
    xacro_file_sim = os.path.join(get_package_share_directory(pkg_name), 'urdf/leo_sim.urdf.xacro')
    robot_description_raw_sim = xacro.process_file(xacro_file_sim).toxml()

    #Not SIM
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf/leo.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Include the t1_sim launch file (only launches if use_sim_time is true)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('t1_sim'), '/launch', '/sim_bringup.launch.py']),
        launch_arguments={}.items(),
    )

   # Launch robot state publisher node with simulation
    robot_state_publisher_sim = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description_raw_sim},
        ],
    )

    # Launch robot state publisher node without simulation
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": False},
            {"robot_description": robot_description_raw},
        ],
    )

    # Publish and broadcast the transform from the odometry frame to the base_link frame
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(pkg_name), 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz', 'nav2.rviz')]
    )

    #RPLIDAR
    node_rplidar = Node(
        node_name='rplidar_composition',
        package='rplidar_ros',
        node_executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
    ),

    # Add actions to LaunchDescription
    ld.add_action(use_sim_time_arg)
    #Use sim_launch only if use_sim_time is true
    # Use robot state publisher sim if sim is true else use robot state publisher
    ld.add_action(OpaqueFunction(function=lambda context: [sim_launch, robot_state_publisher_sim] if context.launch_configurations['use_sim_time'] == 'true' else [robot_state_publisher, node_rplidar]))
    ld.add_action(robot_localization_node)
    ld.add_action(node_rviz)

    return ld

