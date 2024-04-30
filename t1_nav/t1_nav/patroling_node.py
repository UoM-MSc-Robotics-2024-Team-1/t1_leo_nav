import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from .robot_navigator import BasicNavigator, NavigationResult
from std_srvs.srv import SetBool

class PatrolCommand(Enum):
    STOP = 'stop'
    RESUME = 'resume'

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.current_waypoint_index = 0
        self.patrolling = False
        self.marker_pub = self.create_publisher(Marker, 'waypoint_marker', 10)
        self.navigator = BasicNavigator()
        self.waypoints = self.create_waypoints()
        self.srv = self.create_service(SetBool, '/toggle_patrol', self.handle_toggle_patrol)
        self.nav_timer = None  # Timer for periodically checking navigation status

        # Create a dictionary to map commands to methods
        self.command_actions = {
            PatrolCommand.STOP: self.stop_patrol,
            PatrolCommand.RESUME: self.resume_patrol
        }

    # Create posed stamped message
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

    # Create waypoints visualization
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

    # Create waypoints
    def create_waypoints(self):
        """ Generate or retrieve waypoints """
        return [
            self.create_pose_stamped(2.0, 0.0, 0.0, 6.28),
            self.create_pose_stamped(2.0, 1.0, 0.0, 6.28),
            self.create_pose_stamped(0.0, 1.0, 0.0, 6.28),
            # Add more waypoints as needed
        ]
    
    def handle_toggle_patrol(self, request, response):
        self.get_logger().info(f"Handling toggle patrol request (patrolling node): {'start' if request.data else 'stop'}")
        if request.data:
            self.resume_patrol()
        else:
            self.stop_patrol()

        response.success = True
        response.message = 'Patrolling set to: ' + str(self.patrolling)
        return response

    def stop_patrol(self):
        if self.patrolling:
            self.patrolling = False
            self.navigator.cancelNav()
            if self.nav_timer is not None:
                self.nav_timer.cancel()
            self.get_logger().info("Stopping patrol...")

    def resume_patrol(self):
        if not self.patrolling:
            self.patrolling = True
            self.get_logger().info("Resuming patrol...")
            self.patrol()

    def patrol(self):
        if self.current_waypoint_index < len(self.waypoints):
            goal = self.waypoints[self.current_waypoint_index]
            self.navigator.goToPose(goal)
            if self.nav_timer is not None:
                self.nav_timer.cancel()
            self.nav_timer = self.create_timer(0.5, self.check_navigation_status)


    def check_navigation_status(self):
        if self.navigator.isNavComplete():
            self.nav_timer.cancel()  # Stop the timer
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.current_waypoint_index = 0  # Loop back to the first waypoint if looping
            self.patrol()


def main(args=None):
    rclpy.init(args=args)
    patrol_node = PatrolNode()
    rclpy.spin(patrol_node)
    patrol_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
