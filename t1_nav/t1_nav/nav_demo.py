import rclpy
from rclpy.node import Node
import numpy as np

class WaypointsParameterNode(Node):
    def __init__(self):
        super().__init__('waypoints_parameter_node')
        self.declare_parameter('goals', self.create_waypoints_list())

    #def create_waypoints_list(self):
        # Example waypoints list
        #waypoints = [
          #  {'position': {'x': 0.0, 'y': 2.0, 'z': 0.0}, 'orientation': {'yaw': 0.0}},
           # {'position': {'x': 2.0, 'y': 4.0, 'z': 0.0}, 'orientation': {'yaw': 1.57}},
            # Add more waypoints as needed
       # ]
       # return waypoints
    
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    #node = WaypointsParameterNode()
    #rclpy.spin(node)
    #rclpy.shutdown()

if __name__ == '__main__':
    main()
