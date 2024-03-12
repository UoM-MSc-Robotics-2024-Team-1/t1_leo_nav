import rclpy
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
        # Action client for NavigateThroughPoses
        self.action_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')

    def send_waypoints(self, waypoints):
        self.action_client.wait_for_server()
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info(f"Sending waypoints goal with {len(waypoints)} points.")
        self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))


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


    #initial_pose = nav.create_pose_stamped(0.0, 0.0, 0.0, 0.0)
      # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.45
    initial_pose.pose.position.y = 2.15
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
   # navigator.setInitialPose(initial_pose)
    #navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    #navigator.lifecycleStartup()

    # Wait for navigation to fully activate. Use this line if autostart is set to true.
    #navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()
    
    waypoints = [
        nav.create_pose_stamped(2.0, 0.0, 0.0, 1.57),
        nav.create_pose_stamped(2.0, 0.5, 0.0, 1.57),
        nav.create_pose_stamped(0.0, 0.5, 0.0, 1.57),
        # Add more waypoints as needed
    ]

   # sanity check a valid path exists
    # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

    navigator.goThroughPoses(waypoints)

    i = 0
    while not navigator.isNavComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelNav()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=35.0):
                goal_pose4 = PoseStamped()
                goal_pose4.header.frame_id = 'map'
                goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose4.pose.position.x = -5.0
                goal_pose4.pose.position.y = -4.75
                goal_pose4.pose.orientation.w = 0.707
                goal_pose4.pose.orientation.z = 0.707
                navigator.goThroughPoses([goal_pose4])

    # Do something depending on the return code
    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()