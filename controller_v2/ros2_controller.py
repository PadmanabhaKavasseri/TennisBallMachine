import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
import threading
import time
from controller_v2 import socketio, current_mode


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
        
        # Subscribe to person detections
        self.person_subscriber = self.create_subscription(
            Detection2DArray,
            '/yolov5/person_detections',
            self.person_detection_callback,
            10
        )
        self.get_logger().info('Subscribed to /yolov5/person_detections')
        
        # Camera frame parameters (from your detection code)
        self.CAMERA_WIDTH = 1280
        self.CAMERA_HEIGHT = 720
        # Tracking parameters
        self.last_move_time = 0
        self.move_cooldown = 2.0  # 2 seconds between moves
        self.steps_per_move = 100  # Steps to move each time
        
    def person_detection_callback(self, msg):
        """Handle incoming person detections"""
        if len(msg.detections) > 0:
            # Get the first (most confident) person detection
            person = msg.detections[0]
            
            # Extract center coordinates (already in pixels)
            x_center = person.bbox.center.position.x
            y_center = person.bbox.center.position.y
            width = person.bbox.size_x
            height = person.bbox.size_y
            confidence = person.results[0].hypothesis.score
            
            self.get_logger().info(
                f'Person detected: center=({x_center:.1f},{y_center:.1f}) '
                f'size=({width:.1f}x{height:.1f}) conf={confidence:.3f}'
            )

            # Calculate which zone the person is in
            zone = self.calculate_frame_zone(x_center)
            self.get_logger().info(f'Person in zone: {zone}')

            # Auto stepper control (only if in AUTO mode)
            self.handle_auto_stepper_control(zone, x_center)
            
        else:
            self.get_logger().info('No person detected')

    def handle_auto_stepper_control(self, zone, x_center):
        """Move stepper automatically based on person position"""
        # Import current_mode from Flask app
        from controller_v2 import current_mode
        
        if current_mode != "auto":
            return  # Only move in auto mode
            
        # Check cooldown period
        current_time = time.time()
        if current_time - self.last_move_time < self.move_cooldown:
            return  # Too soon since last move
            
        # Determine stepper movement
        if zone == "LEFT":
            # Person on left, move stepper CCW (counter-clockwise)
            self.send_stepper_command("CCW", self.steps_per_move)
            self.get_logger().info(f'AUTO: Moving stepper CCW {self.steps_per_move} steps (person on left)')
            
        elif zone == "RIGHT":
            # Person on right, move stepper CW (clockwise)  
            self.send_stepper_command("CW", self.steps_per_move)
            self.get_logger().info(f'AUTO: Moving stepper CW {self.steps_per_move} steps (person on right)')
            
        else:  # CENTER zone
            self.get_logger().info('AUTO: Person centered, no stepper movement needed')
            return
            
        # Update last move time
        self.last_move_time = current_time
    

    def send_stepper_command(self, direction, steps):
        """Send stepper command with correct Arduino format"""
        try:
            from controller_v2 import arduino
            
            if arduino and arduino.is_open:
                # Correct format: MSG_START|MSG_ID|MOTOR=VALUE|#CHECKSUM|MSG_END
                msg_id = int(time.time() * 1000)
                core_message = f"STEPPER=STEP_{direction}_{steps}"
                base_message = f"MSG_START|{msg_id}|{core_message}"
                
                # Calculate checksum
                checksum = sum(ord(char) for char in base_message) % 256
                full_message = f"{base_message}|#{checksum}|MSG_END"
                
                # Send directly to Arduino
                arduino.write((full_message + '\n').encode())
                arduino.flush()
                
                self.get_logger().info(f'SUCCESS: Sent to Arduino: {full_message}')
                
            else:
                self.get_logger().error('Arduino not connected')
                
        except Exception as e:
            self.get_logger().error(f'Failed to send stepper command: {e}')


    
    def calculate_frame_zone(self, x_center):
        """Determine which zone of the frame the person is in"""
        left_boundary = self.CAMERA_WIDTH * 0.33   # 33% from left
        right_boundary = self.CAMERA_WIDTH * 0.67  # 67% from left
        
        if x_center < left_boundary:
            return "LEFT"
        elif x_center > right_boundary:
            return "RIGHT"
        else:
            return "CENTER"
        
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
