#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MapSaveNode(Node):
    def __init__(self):
        # Initialize the node with the name 'map_save_node'
        super().__init__('map_save_node')
        
        # Create a timer that triggers the call_map_save_service method every 10 seconds
        self.timer = self.create_timer(10.0, self.call_map_save_service)  # Adjust frequency as needed (depedning on the speed of the robot)

    def call_map_save_service(self):
        # Log the action of calling the map save service
        self.get_logger().info('Calling map_save service...')
        
        # Create a client to communicate with the '/map_save' service
        cli = self.create_client(Trigger, '/map_save')
        
        # Create a service request for the Trigger service
        req = Trigger.Request()
        
        # Send the request asynchronously and attach a callback for when the response is received
        future = cli.call_async(req)
        future.add_done_callback(self.callback_map_save)

    def callback_map_save(self, future):
        # This method is called when the service response is received
        try:
            # Get the response from the future
            response = future.result()
            
            # Log the response message from the service
            self.get_logger().info(f'Map save service response: {response.message}')
        except Exception as e:
            # Log an error if the service call fails
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the MapSaveNode
    node = MapSaveNode()
    
    try:
        # Keep the node running and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle keyboard interrupt (Ctrl+C) gracefully
        pass
    finally:
        # Cleanup and shutdown the node
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # Execute the main function when the script is run directly
    main()
