import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from t1_nav.srv import PatrolCommand
from .robot_navigator import BasicNavigator, NavigationResult  # Helper module

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.patrol_service = self.create_service(PatrolCommand, 'patrol_command', self.patrol_command_callback)
        self.current_waypoint_index = 0
        self.patrolling = False
        self.marker_pub = self.create_publisher(Marker, 'waypoint_marker', 10)
        self.navigator = BasicNavigator()
        self.waypoints = self.create_waypoints()

    def create_pose_stamped(self, x, y, z, yaw, frame_id='map'):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def publish_waypoints(self):
        """ Publish waypoints for visualization """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.scale.x = 0.2  # Sphere radius
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.points = [wp.pose.position for wp in self.waypoints]
        self.marker_pub.publish(marker)

    def create_waypoints(self):
        """ Generate or retrieve waypoints """
        return [
            self.create_pose_stamped(2.0, 0.0, 0.0, 0.0),
            self.create_pose_stamped(2.0, 1.0, 0.0, 0.0),
            self.create_pose_stamped(0.0, 1.0, 0.0, 0.0),
            # Add more waypoints as needed
        ]

    def patrol_command_callback(self, request, response):
        if request.command == "start":
            self.patrolling = True
            self.get_logger().info("Starting patrol...")
            self.patrol()

        elif request.command == "stop":
            self.patrolling = False
            self.navigator.cancelNav()
            self.get_logger().info("Stopping patrol...")

        elif request.command == "resume":
            self.patrolling = True
            self.get_logger().info("Resuming patrol...")
            self.patrol()

        response.success = True
        response.message = "Command executed: " + request.command
        return response

    def patrol(self):

        while self.patrolling and self.current_waypoint_index < len(self.waypoints):

            goal = self.waypoints[self.current_waypoint_index]
            # self.publisher.publish(goal)
            self.navigator.goToPose(goal)

            while not self.navigator.isNavComplete():
                print('Moving To Goal ' + str(self.current_waypoint_index+1) + ' out of ' + str(len(self.waypoints)))
                # Simulate reaching the goal
                rclpy.spin_once(self, timeout_sec=1)  # This is a placeholder for real navigation logic

            self.current_waypoint_index += 1

def main(args=None):
    rclpy.init(args=args)
    patrol_node = PatrolNode()
    rclpy.spin(patrol_node)
    patrol_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
