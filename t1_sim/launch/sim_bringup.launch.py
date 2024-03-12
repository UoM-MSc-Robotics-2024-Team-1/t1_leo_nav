import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    ld = LaunchDescription()

    # Specify the name of the package
    pkg_name = 't1_sim'

    # Set ignition resource path (so it can find your world files)
    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
    value=[os.path.join(get_package_share_directory(pkg_name),'worlds')])

    # Include extra models in the world
    sdf_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'boundary.sdf')

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        gz_world_path =  os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + os.path.join(get_package_share_directory(pkg_name), "worlds")
    else:
        gz_world_path =  os.path.join(get_package_share_directory(pkg_name), "worlds")

    ign_resource_path_update = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',value=[gz_world_path])

    # Include the Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
    launch_arguments={
    'gz_args' : '-r empty.sdf'
    }.items(),
    )

    # Add features
    gz_spawn_objects = Node(package='ros_gz_sim', executable='create',
    arguments=['-file', sdf_path,
    '-x', '2.0',
    '-y', '0.5',
    '-z', '0.0'],
    output='screen'
    )

    #GAZEBO

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

    # Add actions
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path)
    ld.add_action(ign_resource_path_update)
    ld.add_action(launch_gazebo)
    ld.add_action(gz_spawn_objects)
    ld.add_action(leo_rover)
    ld.add_action(topic_bridge)

    return ld