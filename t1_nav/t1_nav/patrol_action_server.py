import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from example_interfaces.srv import SetPose, GetPose
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from action_msgs.msg import GoalStatus
from .robot_navigator import BasicNavigator, NavigationResult  # Helper module

class PatrolActionServer(Node):
    def __init__(self):
        super().__init__('patrol_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.marker_pub = self.create_publisher(Marker, 'waypoint_marker', 10)
        self.navigator = BasicNavigator()
        self.waypoints = self.create_waypoints()

        # Services
        self.add_waypoint_service = self.create_service(SetPose, 'add_waypoint', self.add_waypoint_callback)
        self.get_waypoint_service = self.create_service(GetPose, 'get_current_waypoint', self.get_current_waypoint_callback)

    # Service callback to get the current waypoint
    def get_current_waypoint_callback(self, request, response):
        # This could return the next waypoint to navigate to
        if self.current_waypoint_index < len(self.waypoints):
            response.pose = self.waypoints[self.current_waypoint_index]
            response.success = True
        else:
            response.success = False  # No more waypoints
        return response

    # Service callback to add a new waypoint
    def add_waypoint_callback(self, request, response):
        new_waypoint = self.create_pose_stamped(request.x, request.y, request.z, request.yaw)
        self.waypoints.append(new_waypoint)
        response.success = True
        return response
    
    # Gets called when the action server receives a new goal
    def execute_callback(self, goal_handle):
        target_pose = goal_handle.request.pose
        self.get_logger().info(f'Navigating to pose: {target_pose.pose.position.x}, {target_pose.pose.position.y}')
        self.navigator.goToPose(target_pose)

        # Wait for the navigation to complete
        while not self.navigator.isNavComplete():
            rclpy.spin_once(self, timeout_sec=0.1)  # Keep processing callbacks
            print('Moving to goal...')
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Navigation canceled.')
                return NavigateToPose.Result()

        goal_handle.succeed()
        self.current_waypoint_index += 1 # Update to the next waypoint
        self.get_logger().info('Navigation succeeded!')
        return NavigateToPose.Result()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received new goal request.')
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancellation request.')
        self.navigator.cancelNav()  # Assuming you have a cancel method in BasicNavigator
        return rclpy.action.server.CancelResponse.ACCEPT

    def create_waypoints(self):
        """ Generate or retrieve waypoints """
        return [
            self.create_pose_stamped(2.0, 0.0, 0.0, 0.0),
            self.create_pose_stamped(2.0, 1.0, 0.0, 0.0),
            self.create_pose_stamped(0.0, 1.0, 0.0, 0.0),
            # Add more waypoints as needed
        ]

    def add_waypoint(self, x, y, z, yaw):
        """ Add a new waypoint to the list """
        self.waypoints.append(self.create_pose_stamped(x, y, z, yaw))

    def remove_waypoint(self, index):
        """ Remove a waypoint from the list """
        if index < len(self.waypoints):
            del self.waypoints[index]

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

def main(args=None):
    rclpy.init(args=args)
    patrol_action_server = PatrolActionServer()
    patrol_action_server.publish_waypoints()  # Visualize the initial waypoints

    # Spin in a separate thread to keep the service responsive
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(patrol_action_server, executor=executor)

if __name__ == '__main__':
    main()
