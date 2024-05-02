import rclpy
from rclpy.node import Node
import numpy as np
import transforms3d as t3d
from std_srvs.srv import SetBool

class ManipualtorNode(Node):
    def __init__(self):
        super().__init__('manipulator_node')
        self.cli = self.create_client(SetBool, '/toggle_manipulator')
 
    def stop_manipulator_command(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
 
        request = SetBool.Request()
        request.data = False
        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_service_response)
        self.get_logger().info("Sent Manipulator command: stop")
 
    def handle_service_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

def main(args=None):
    rclpy.init(args=args)
    manipulator_node = ManipualtorNode()
    rclpy.spin(manipulator_node)
    manipulator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
  
