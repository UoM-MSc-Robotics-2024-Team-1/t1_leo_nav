"""
Autonomously explore an environment using breadth-first search.

Using the nubot, a differential-drive robot, this node with use
the breadth-first algorithm to autonomously explore and map its
environment.

PUBLISHERS:
  + /goal_pose (PoseStamped) - Tell nav2 where nubot should drive to.
  + /visualization_marker (Marker) - Visualize the breadth-first algo.

SUBSCRIBERS:
  + /map (OccupancyGrid) - A of the environment created by the slam_toolbox.
  + /rosout (Log) - A hacky way to receive status messages from nav2.

"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from nav_msgs.msg import OccupancyGrid

from std_msgs.msg import Header

from rcl_interfaces.msg import Log

from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from visualization_msgs.msg import Marker

import numpy as np
from enum import Enum, auto


class State(Enum):
    WAITING = auto()
    FIND_ROBOT = auto()
    FIND_FRONTIER = auto()
    GO_TO_FRONTIER = auto()


class Explore(Node):
    def __init__(self):
        super().__init__("explore")

        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        # create a timer
        self.timer = self.create_timer(
            0.01, self.timer_callback,
            callback_group=self.timer_callback_group)

        # create subscriber
        self.occupancy_grid_sub = self.create_subscription(
            OccupancyGrid, "/map", self.occupancy_grid_callback, 10)

        self.log_sub = self.create_subscription(
            Log, "/rosout", self.log_callback, 10)

        # create publisher
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, "/goal_pose", 10)

        self.marker_pub = self.create_publisher(
            Marker, "/visualization_marker", 10)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # create variables
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid_data = None
        self.original_costmap_size = None
        self.original_occupancy_grid_data = None

        self.robot_location = None
        self.goal_pose_stamped = None

        # variables for breadth first search
        self.child_positions = [
            [0, -2],
            [-2, 0],
            [2, 0],
            [0, 2],
        ]

        self.visited = []
        self.fringe = []  # the cells the breadth first search can use

        self.state = State.FIND_ROBOT

    def get_transform(self, parent_frame, child_frame):
        """
        Listen to transforms between parent and child frame.

        Args:
        ----
        parent_frame (string): name of parent frame
        child_frame (string): name of child frame

        Returns
        -------
        translation (Point): The position vector of the child frame
        in the parent frame, inside a Point() object.
        rotation (Quaternion): The quaternion associated with the
        position vector.

        """
        try:
            trans = self.buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time()
            )
            transl = trans.transform.translation
            rot = trans.transform.rotation
            translation = Point(x=transl.x, y=transl.y, z=transl.z)
            rotation = Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w)

            return translation, rotation

        except Exception:
            return Point(), Quaternion()

    def occupancy_grid_callback(self, msg):
        """Receive the occupancy grid message from slam_toolbox."""
        self.occupancy_grid = msg
        self.occupancy_grid_data = np.asarray(self.occupancy_grid.data)
        self.occupancy_grid_data = self.occupancy_grid_data.reshape(
            (self.occupancy_grid.info.height, self.occupancy_grid.info.width))

    def log_callback(self, msg):
        """Receive status messages from /rosout."""
        log = Log()
        log.msg = msg.msg

        if log.msg == "Reached the goal!":
            self.state = State.FIND_ROBOT

        # not sure which failure message is the right one, so
        # let's account for both
        elif log.msg == "Goal failed":
            self.state = State.FIND_ROBOT

        elif log.msg == "spin failed":
            self.state = State.FIND_ROBOT

    def determine_frontier(self, index):
        """
        Determine whether a singular frontier cell is adequate.

        Determine whether a cell in the OccupancyGrid is part of
        a valid frontier cluster/blob. Essentially, I create a square
        surrounding the frontier cell and check how many other frontier
        and wall cells there are in its surrounding area. If there are
        too many walls or not enough frontier cells, then it isn't safe
        or worth it to move the nubot there.

        Args:
        ----
        index (int[]): An index of the OccupancyGrid.

        Returns
        -------
        (bool): Whether or not the frontier cell is valid.

        """
        num_frontiers = 1
        num_walls = 0
        grid_length = 20
        wall_threshold = 0

        start_posx = index[0] - grid_length/2
        start_posy = index[1] - grid_length/2

        self.get_logger().info(f"start_posx: {start_posx}")
        self.get_logger().info(f"start_posy: {start_posy}")

        for i in range(grid_length):
            for j in range(grid_length):
                child_position_row = int(start_posx + i)
                child_position_column = int(start_posy + j)
                child_position = (child_position_row,
                                  child_position_column)

                self.get_logger().info(f"child_position: {child_position}")

                if child_position not in self.visited and \
                        self.original_occupancy_grid_data[
                            child_position_row][child_position_column] < 0:
                    num_frontiers += 1
                if self.original_occupancy_grid_data[
                        child_position_row][child_position_column] == 100:
                    num_walls += 1
                    if num_walls > wall_threshold:
                        return False

        if num_frontiers > 2:
            return True
        else:
            return False

    def map_coords_to_occupancy_grid(self, point):
        """
        Convert x and y coordinates to OccupancyGrid indexes.

        Convert x and y coordinates in the world frame to
        OccupancyGrid coordinates using the resolution of
        OccupancyGrid cells and the dimensions of the costmap.

        Args:
        ----
        point (Point): The current location of the robot in the world frame.

        Returns
        -------
        robot_location (int[]): The location of the robot in the costmap.

        """
        col_index = int((point.x - self.original_costmap_size[0]) /
                        self.occupancy_grid.info.resolution)
        row_index = int((point.y - self.original_costmap_size[1]) /
                        self.occupancy_grid.info.resolution)

        robot_location = [row_index, col_index]

        return robot_location

    def occupancy_grid_to_map_coords(self, index):
        """
        Convert OccupancyGrid indexes to x and y coordinates.

        Convert OccupancyGrid indexes to x and y coordinates
        in the world frame. Additionally, calculate the quaternion
        from the current location of the robot to the goal pose.

        Args:
        ----
        index (int[]): An index of the OccupancyGrid.

        Returns
        -------
        pose (Pose): Coordinates in a pose variable corresponding
        to the OccupancyGrid index.

        """
        pose_x = index[1] * self.occupancy_grid.info.resolution + \
            self.original_costmap_size[0]
        pose_y = index[0] * self.occupancy_grid.info.resolution + \
            self.original_costmap_size[1]

        pose = PoseStamped(pose=Pose(position=Point(
            x=pose_x, y=pose_y)))

        current_point, current_quat = self.get_transform(
            'map', 'base_link')

        robot = np.array([current_point.x, current_point.y])
        goal = np.array([pose.pose.position.x,
                        pose.pose.position.y])

        direction = robot - goal
        unit_vector = direction / np.linalg.norm(direction)

        quaternion = Quaternion(
            x=unit_vector[1], y=unit_vector[0], z=0.0, w=0.0)

        pose.pose.orientation = quaternion

        return pose

    def check_distance_from_robot(self, child_pose):
        """
        Check the distance between a point and the robot.

        Checks the distance between a point and the location
        of the robot's base_link in the map frame. Used for the
        breadth-first search.

        Args:
        ----
        child_pose (Pose): The point that we want to check.

        Returns
        -------
        distance (float): The distance between the point and the nubot.

        """
        current_point, current_quat = self.get_transform(
            'map', 'base_link')

        distance = np.linalg.norm(np.asarray(
            [current_point.x, current_point.y]) - np.asarray(
                [child_pose.pose.position.x, child_pose.pose.position.y]))

        return distance

    def get_marker(self, pose_stamped):
        """Create a spherical marker."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0

        return marker

    def timer_callback(self):
        """
        Execute the breadth first search algo and send goal_poses.

        Execute the breadth first search algorithm and find a
        detect the closest valid frontier to the robot for it to
        go and explore. Also send /goal_poses to nav2 for it to plan.

        Args:
        ----
        None

        Returns
        -------
        None

        """
        if self.state == State.FIND_ROBOT:

            if self.occupancy_grid_data is None:
                return

            self.original_costmap_size = [
                self.occupancy_grid.info.origin.position.x,
                self.occupancy_grid.info.origin.position.y]
            self.original_occupancy_grid_data = self.occupancy_grid_data

            robot_map_point, _ = self.get_transform(
                'map', 'base_link')

            self.robot_location = self.map_coords_to_occupancy_grid(
                robot_map_point)

            self.state = State.FIND_FRONTIER

        elif self.state == State.FIND_FRONTIER:

            i = self.robot_location[0]  # row
            j = self.robot_location[1]  # column

            self.fringe = [(i, j)]

            for cell in self.fringe:
                is_frontier = False

                for modifier in self.child_positions:
                    # change the location of the cell in focus to one of its
                    # children

                    child_position_row = cell[0] + modifier[0]
                    child_position_column = cell[1] + \
                        modifier[1]
                    child_position = (child_position_row,
                                      child_position_column)

                    child_pose = self.occupancy_grid_to_map_coords(
                        child_position)

                    far_away = self.check_distance_from_robot(child_pose)

                    if child_position not in self.visited and child_position \
                            not in self.fringe:

                        if self.original_occupancy_grid_data[
                            child_position_row][
                            child_position_column] == -1 and \
                                far_away > 1.5:
                            if not is_frontier:
                                is_frontier = self.determine_frontier(
                                    child_position)

                        elif self.original_occupancy_grid_data[
                                child_position_row][
                                    child_position_column] == 0:
                            # fringe is a list of cells where I care about
                            # their children

                            marker = self.get_marker(
                                self.occupancy_grid_to_map_coords(
                                    child_position))

                            self.marker_pub.publish(marker)
                            self.fringe.append(child_position)

                        elif self.original_occupancy_grid_data[
                                child_position_row][
                                    child_position_column] == 100:

                            self.visited.append(child_position)

                        else:
                            self.fringe.append(child_position)

                if is_frontier:

                    marker = self.get_marker(self.occupancy_grid_to_map_coords(
                        child_position))
                    self.marker_pub.publish(marker)

                    # convert the indices back to a pose in the map frame
                    self.goal_pose_stamped = self.occupancy_grid_to_map_coords(
                        child_position)

                    self.state = State.GO_TO_FRONTIER
                    self.fringe.clear()
                    self.visited.clear()
                    return
                self.visited.append(cell)

        elif self.state == State.GO_TO_FRONTIER:

            self.goal_pose_stamped.header = Header(
                stamp=self.get_clock().now().to_msg())
            self.goal_pose_stamped.header.frame_id = "map"

            self.goal_pose_pub.publish(self.goal_pose_stamped)
            self.state = State.WAITING

        elif self.state == State.WAITING:
            marker = self.get_marker(self.goal_pose_stamped)
            self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    explore = Explore()

    rclpy.spin(explore)


if __name__ == '__main__':
    main()