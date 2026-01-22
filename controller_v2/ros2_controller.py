import rclpy
from rclpy.node import Node
import threading
import time

class TennisBallControllerNode(Node):
    def __init__(self):
        super().__init__('tennis_ball_controller')
        self.get_logger().info('Tennis Ball Controller ROS2 Node started')
        
        # Start Flask app in separate thread
        self.flask_thread = threading.Thread(target=self.run_flask_app, daemon=True)
        self.flask_thread.start()
        
        # Give Flask a moment to start
        time.sleep(2)
        self.get_logger().info('Flask web server should be running on http://0.0.0.0:5000')
        
        # TODO: Add bbox subscriber here later
        
    def run_flask_app(self):
        """Run the Flask controller in a separate thread"""
        try:
            from controller_v2 import start_controller
            start_controller()
        except Exception as e:
            self.get_logger().error(f'Failed to start Flask app: {e}')

def main():
    rclpy.init()
    node = TennisBallControllerNode()
    
    try:
        node.get_logger().info('ROS2 node spinning... Press Ctrl+C to stop')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
