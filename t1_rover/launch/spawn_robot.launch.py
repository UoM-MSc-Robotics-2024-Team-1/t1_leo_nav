# Copyright 2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch import LaunchDescription



def spawn_robot(context: LaunchContext, namespace: LaunchConfiguration):

    robot_ns = context.perform_substitution(namespace)

    pkg_name = 't1_rover'
    file_subpath = 'urdf/leo_sim.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file,  mappings={"robot_ns": robot_ns}).toxml()


    if robot_ns == "":
        robot_gazebo_name = "leo_rover"
        node_name_prefix = ""
    else:
        robot_gazebo_name = "leo_rover_" + robot_ns
        node_name_prefix = robot_ns + "_"


    # Launch robot state publisher node
    robot_state_publisher = Node(
        namespace=robot_ns,
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
        namespace=robot_ns,
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_sim_create",
        output="both",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            robot_gazebo_name,
            "-z",
            "1.65",
        ],
    )

    # Add ros2 launch SLAM toolbox 
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'scan_topic': robot_ns + '/scan',
            'odom_topic': robot_ns + '/odom',
            'map_frame': 'map',
            'base_frame': robot_ns + 'base_link',
            'global_frame': 'map',
            }.items()
    )

 # Define the full path for the map file
    map_file_path = os.path.join(get_package_share_directory('t1_rover'), 'maps', 'my_map')

    map_saver_params = {
        'map_topic': 'map',
        'map_url': 'map_name',  # Ensure this is a valid path or adjust as needed
        'image_format': 'pgm',
        'map_mode': 'trinary',
        'free_thresh': 0.25,
        'occupied_thresh': 0.65
    }

    node_map_saver = Node(
        package='t1_rover',
        executable='custom_map_saver',
        name='map_saver',
        output='screen',
        parameters=[map_saver_params]  # Pass parameters here
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=node_name_prefix + "parameter_bridge",
        arguments=[
            robot_ns + '/cmd_vel'                               +   '@geometry_msgs/msg/Twist'        +   ']'   +   'ignition.msgs.Twist',
            robot_ns + '/odom'                                  +   '@nav_msgs/msg/Odometry'          +   '['   +   'ignition.msgs.Odometry',
            robot_ns + '/tf'                                    +   '@tf2_msgs/msg/TFMessage'         +   '['   +   'ignition.msgs.Pose_V',
            robot_ns + '/imu/data_raw'                          +   '@sensor_msgs/msg/Imu'            +   '['   +   'ignition.msgs.IMU',
            robot_ns + '/camera/camera_info'                    +   '@sensor_msgs/msg/CameraInfo'     +   '['   +   'ignition.msgs.CameraInfo',
            robot_ns + '/joint_states'                          +   '@sensor_msgs/msg/JointState'     +   '['   +   'ignition.msgs.Model',
            robot_ns + '/scan'                                  +   '@sensor_msgs/msg/LaserScan'      +   '['   +   'ignition.msgs.LaserScan',
            '/world/empty/' + robot_ns + 'joint_state'          +   '@sensor_msgs/msg/JointState'     +   '['   +   'ignition.msgs.Model',
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        remappings= [
            ('/world/empty/' + robot_ns + 'joint_state', 'joint_states')
            ],
        output="screen",
    )

    # Camera image bridge
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name=node_name_prefix + "image_bridge",
        arguments=[robot_ns + "/camera/image_raw"],
        output="screen",
    )
    return [
        robot_state_publisher,
        leo_rover,
        slam_toolbox_launch,
        node_map_saver,
        topic_bridge,
        image_bridge,
    ]


def generate_launch_description():

    ld = LaunchDescription()

    name_argument = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )

    namespace = LaunchConfiguration("robot_ns")

    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('t1_sim'), '/launch', '/sim_bringup.launch.py']),
        launch_arguments={}.items(),
    )

    '''
    # Including the laser launch file 
    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('t1_rover'), '/launch', '/laser.launch']),
        launch_arguments={}.items(),
    )
    '''



    # Add actions to LaunchDescription
    ld.add_action(name_argument)
    ld.add_action(OpaqueFunction(function=spawn_robot, args=[namespace]))
    ld.add_action(gazebo_launch)
    #ld.add_action(laser_launch)

    return ld