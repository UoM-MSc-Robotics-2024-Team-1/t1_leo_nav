import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf_transformations import quaternion_from_euler
from t1_nav.robot_navigator import BasicNavigator, NavigationResult

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        # Marker publisher for RViz2 visualization
        self.marker_pub = self.create_publisher(Marker, 'waypoint_marker', 10)

    def create_pose_stamped(self, x, y, z, yaw, frame_id='map'):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position = Point(x=x, y=y, z=z)
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def publish_waypoints(self, waypoints):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5  # Sphere radius
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Alpha
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for wp in waypoints:
            p = Point()
            p.x = wp.pose.position.x
            p.y = wp.pose.position.y
            p.z = wp.pose.position.z
            marker.points.append(p)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init()
    nav = SimpleNavigator()
    navigator = BasicNavigator()

    waypoints = [
        nav.create_pose_stamped(2.0, 0.0, 0.0, 0.0),
        nav.create_pose_stamped(2.0, 1.0, 0.0, 0.0),
        nav.create_pose_stamped(0.0, 1.0, 0.0, 0.0),
        # Add more waypoints as needed
    ]

    nav.publish_waypoints(waypoints)  # Visualize waypoints in RViz2

    i = 0
    navigator.goToPose(waypoints[i])
    while i < len(waypoints):
        navigator.goToPose(waypoints[i])
        while not navigator.isNavComplete():
            print('Moving To Goal ' + str(i+1) + ' out of ' + str(len(waypoints)))
            time.sleep(2)
        print('Goal succeeded!')
        i += 1

    print('Navigation Complete!')
    navigator.lifecycleShutdown()

if __name__ == '__main__':
    main()
