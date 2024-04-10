import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration # Handles time for ROS 2
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateThroughPoses
from tf_transformations import quaternion_from_euler
from .robot_navigator import BasicNavigator, NavigationResult # Helper module

class SimpleNavigator(Node):

    def __init__(self):
        super().__init__('simple_navigator')

    def create_pose_stamped(self, x, y, z, yaw, frame_id='map'):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()  # Corrected time assignment
        pose.header.frame_id = frame_id
        pose.pose.position = Point(x=x, y=y, z=z)
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return pose

def main(args=None):
    # Start the ROS 2 Python Client Library 
    rclpy.init()

    nav = SimpleNavigator()
    # Launch the ROS 2 Navigation Stack
    navigator = BasicNavigator()


    waypoints = [
        nav.create_pose_stamped(2.0, 0.0, 0.0, 0.0),
        nav.create_pose_stamped(2.0, 0.5, 0.0, 0.0),
        nav.create_pose_stamped(0.0, 0.5, 0.0, 0.0),
        nav.create_pose_stamped(0.0, 1.0, 0.0, 0.0),
        nav.create_pose_stamped(1.0, 1.0, 0.0, 0.0),
        nav.create_pose_stamped(1.0, 1.5, 0.0, 0.0),
        nav.create_pose_stamped(0.0, 1.5, 0.0, 0.0),
        nav.create_pose_stamped(0.0, 1.5, 0.0, 0.0),
        # Add more waypoints as needed
    ]

    i = 0

     # Start the robot towards the first waypoint
    navigator.goToPose(waypoints[i])
    i += 1

    while not navigator.isNavComplete() and i < len(waypoints):
        if navigator.isNavComplete():
            navigator.goToPose(waypoints[i])
            print('Goal succeeded!')
            i += 1
            time.sleep(2)
        else:
            print('Moving To Goal...')
    

    #znavigator.lifecycleShutdown()


if __name__ == '__main__':
    main()