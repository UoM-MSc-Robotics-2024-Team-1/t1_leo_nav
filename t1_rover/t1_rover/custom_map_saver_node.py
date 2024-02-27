import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap
import time

class CustomMapSaverNode(Node):
    def __init__(self):
        super().__init__('custom_map_saver_node')
        self.client = self.create_client(SaveMap, '/map_saver/save_map')

        # Declare parameters with default values
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('map_url', 'map_name')
        self.declare_parameter('image_format', 'pgm')
        self.declare_parameter('map_mode', 'trinary')
        self.declare_parameter('free_thresh', 0.25)
        self.declare_parameter('occupied_thresh', 0.65)

    def save_map(self):
        # Fetch parameters ERROR HERE WITH .GET_VALUE()---------------------------------------------------------sS
        map_topic = self.get_parameter('map_topic').get_value()
        map_url = self.get_parameter('map_url').get_value()
        image_format = self.get_parameter('image_format').get_value()
        map_mode = self.get_parameter('map_mode').get_value()
        free_thresh = self.get_parameter('free_thresh').get_value()
        occupied_thresh = self.get_parameter('occupied_thresh').get_value()

        # Ensure the service is available
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service /map_saver/save_map not available')
            return
        
        # Prepare and send the request
        request = SaveMap.Request()
        request.map_topic = map_topic
        request.map_url = map_url
        request.image_format = image_format
        request.map_mode = map_mode
        request.free_thresh = free_thresh
        request.occupied_thresh = occupied_thresh
        self.future = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = CustomMapSaverNode()

    node.get_logger().info('Waiting for 5 seconds before saving the map...')
    time.sleep(5)  # Initial delay

    # Attempt to save the map using parameters
    node.get_logger().info('Attempting to save the map...')
    node.save_map()
    
    rclpy.spin_until_future_complete(node, node.future)
    
    # Handle the response
    try:
        response = node.future.result()
        if response.success:
            node.get_logger().info('Map successfully saved.')
        else:
            node.get_logger().error('Failed to save map. Service call completed, but returned failure.')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
