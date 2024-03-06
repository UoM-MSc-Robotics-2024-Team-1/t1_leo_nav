import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import Pose, Point, Quaternion
from builtin_interfaces.msg import Time
from tf_transformations import quaternion_from_euler

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        # Action client for NavigateThroughPoses
        self.action_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')

    def send_waypoints(self, waypoints):
        self.action_client.wait_for_server()
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info(f"Sending waypoints goal with {len(waypoints)} points.")
        return self.action_client.send_goal_async(goal_msg)
    

    def create_pose_stamped(self, x, y, z, yaw, frame_id="map"):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()  # Corrected time assignment
        pose.header.frame_id = frame_id
        
        pose.pose.position = Point(x=x, y=y, z=z)
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return pose

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()
    
    waypoints = [
        navigator.create_pose_stamped(0.0, 1.0, 0.0, 0.0),
        navigator.create_pose_stamped(-1.0, 1.0, 0.0, 1.57),
        # Add more waypoints as needed
    ]
    
    # Send the waypoints to the action server and wait for the action to complete
    future = navigator.send_waypoints(waypoints)
    rclpy.spin_until_future_complete(navigator, future)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
