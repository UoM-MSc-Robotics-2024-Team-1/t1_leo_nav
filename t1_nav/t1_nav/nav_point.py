import rclpy
from rclpy.action import ActionClient
import time
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from builtin_interfaces.msg import Time
from tf_transformations import quaternion_from_euler


class NavPoint(Node):
    def __init__(self):
        super().__init__(node_name='nav_point')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    
    def publish_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    nav_point = NavPoint()
    nav_point.publish_twist(0.5, 0.0)  # Move the robot at 0.5 m/s
    time.sleep(2)  # Wait for 2 seconds
    nav_point.publish_twist(0.0, 0.0)  # Stop the robot
    rclpy.spin(nav_point)
    rclpy.shutdown()

if __name__ == '__main__':
    main()