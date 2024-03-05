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
    file_subpath = 'urdf/leo_sim.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Include the t1_sim launch file
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('t1_sim'), '/launch', '/sim_bringup.launch.py']),
        launch_arguments={}.items(),
    )

    # Launch robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description_raw},
        ],
    )

    # Spawn a robot inside a simulation
    leo_rover = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-z",
            "0.5",
        ],
        output="screen",
    )

     # Bridge
    # Bridge ROS topics and Gazebo messages for establishing communication
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        arguments=[
            '/clock'                                 +   '@rosgraph_msgs/msg/Clock'        +   '['   +   'ignition.msgs.Clock',
            '/cmd_vel'                               +   '@geometry_msgs/msg/Twist'        +   '@'   +   'ignition.msgs.Twist',
            '/odom'                                  +   '@nav_msgs/msg/Odometry'          +   '['   +   'ignition.msgs.Odometry',
            '/tf'                                    +   '@tf2_msgs/msg/TFMessage'         +   '['   +   'ignition.msgs.Pose_V',
            '/imu/data_raw'                          +   '@sensor_msgs/msg/Imu'            +   '['   +   'ignition.msgs.IMU',
            '/camera/camera_info'                    +   '@sensor_msgs/msg/CameraInfo'     +   '['   +   'ignition.msgs.CameraInfo',
            '/joint_states'                          +   '@sensor_msgs/msg/JointState'     +   '['   +   'ignition.msgs.Model',
            '/scan'                                  +   '@sensor_msgs/msg/LaserScan'      +   '['   +   'ignition.msgs.LaserScan',
            '/world/empty/joint_state'               +   '@sensor_msgs/msg/JointState'     +   '['   +   'ignition.msgs.Model',
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        remappings= [
            ('/world/empty/joint_state', 'joint_states')
            ],
        output="screen",
    )


    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(sim_launch)
    ld.add_action(leo_rover)
    ld.add_action(robot_state_publisher)
    ld.add_action(topic_bridge)

    return ld

