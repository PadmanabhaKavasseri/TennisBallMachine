#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import time

from tennis_ball_msgs.msg import MotorCommand

class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__('person_tracker')
        
        # Subscribe to person detections
        self.person_subscriber = self.create_subscription(
            Detection2DArray,
            '/yolov5/person_detections',
            self.person_detection_callback,
            10
        )
        
        # Publish Arduino commands
        self.motor_command_publisher = self.create_publisher(MotorCommand, '/motor/command', 10)
        
        # Tracking parameters
        self.camera_width = 1280
        self.camera_height = 720
        self.last_move_time = 0
        self.move_cooldown = 1.0  # 2 seconds between moves
        self.steps_per_move = 100  # Steps to move each time
        
        # Frame zones
        self.left_boundary = self.camera_width * 0.33   # 33% from left
        self.right_boundary = self.camera_width * 0.67  # 67% from left
        
        self.get_logger().info('Person Tracker Node started')
        self.get_logger().info(f'Camera resolution: {self.camera_width}x{self.camera_height}')
        self.get_logger().info(f'Left zone: 0 - {self.left_boundary:.0f}')
        self.get_logger().info(f'Center zone: {self.left_boundary:.0f} - {self.right_boundary:.0f}')
        self.get_logger().info(f'Right zone: {self.right_boundary:.0f} - {self.camera_width}')
    
    def person_detection_callback(self, msg):
        """Handle incoming person detections"""
        if len(msg.detections) > 0:
            # Get the first (most confident) person detection
            person = msg.detections[0]
            
            # Extract center coordinates (already in pixels)
            x_center = person.bbox.center.position.x
            y_center = person.bbox.center.position.y
            confidence = person.results[0].hypothesis.score
            
            # Calculate which zone the person is in
            zone = self.calculate_frame_zone(x_center)
            
            self.get_logger().info(
                f'👤 Person detected: center=({x_center:.1f},{y_center:.1f}) '
                f'zone={zone} conf={confidence:.3f}'
            )
            
            # Move stepper based on person position
            self.track_person(zone, x_center)
            
        else:
            self.get_logger().info('No person detected')
    
    def calculate_frame_zone(self, x_center):
        """Determine which zone of the frame the person is in"""
        if x_center < self.left_boundary:
            return "LEFT"
        elif x_center > self.right_boundary:
            return "RIGHT"
        else:
            return "CENTER"
    
    def track_person(self, zone, x_center):
        """Move stepper to track person"""
        # Check cooldown period
        current_time = time.time()
        if current_time - self.last_move_time < self.move_cooldown:
            self.get_logger().info(f'⏳ Cooldown active ({current_time - self.last_move_time:.1f}s)')
            return
        
        # Determine movement
        if zone == "LEFT":
            # Person on left, move stepper CCW (counter-clockwise)
            self.send_stepper_command("CCW", self.steps_per_move)
            self.get_logger().info(f'⬅️  Moving stepper CCW {self.steps_per_move} steps (person on left)')
            
        elif zone == "RIGHT":
            # Person on right, move stepper CW (clockwise)
            self.send_stepper_command("CW", self.steps_per_move)
            self.get_logger().info(f'➡️  Moving stepper CW {self.steps_per_move} steps (person on right)')
            
        else:  # CENTER zone
            self.get_logger().info('🎯 Person centered - no movement needed')
            return
        
        # Update last move time
        self.last_move_time = current_time
    
    def send_stepper_command(self, direction, steps):
        cmd = MotorCommand()
        cmd.motor = "stepper"
        cmd.command = "move_steps"
        cmd.value = float(steps)
        cmd.direction = direction.lower()  # "cw" or "ccw"
        cmd.speed = 0.0
        
        self.motor_command_publisher.publish(cmd)

def main():
    rclpy.init()
    node = PersonTrackerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down person tracker...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
