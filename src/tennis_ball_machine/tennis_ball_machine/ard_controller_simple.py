#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading
import os

class SimpleArduinoController(Node):
    def __init__(self):
        super().__init__('simple_arduino_controller')
        
        # Arduino connection
        self.arduino = None
        self.connect_arduino()
        
        # Simple command subscriber
        self.command_subscriber = self.create_subscription(
            String,
            '/arduino/raw_command',
            self.command_callback,
            10
        )
        
        # Response publisher
        self.response_publisher = self.create_publisher(
            String,
            '/arduino/response',
            10
        )
        
        self.get_logger().info('Simple Arduino Controller started')
        self.get_logger().info('Send commands to: /arduino/raw_command')
        self.get_logger().info('Example: ros2 topic pub --once /arduino/raw_command std_msgs/String "data: \'STEPPER=ENABLE\'"')
    
    def connect_arduino(self):
        """Connect to Arduino"""
        if os.path.exists('/dev/ttyACM0'):
            try:
                self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=2)
                time.sleep(3)  # Wait for Arduino reset
                self.arduino.reset_input_buffer()
                self.get_logger().info("✅ Arduino connected on /dev/ttyACM0")
                
                # Start response monitor
                threading.Thread(target=self.monitor_arduino, daemon=True).start()
                
            except Exception as e:
                self.get_logger().error(f"❌ Arduino connection failed: {e}")
        else:
            self.get_logger().error("❌ /dev/ttyACM0 not found")
    
    def monitor_arduino(self):
        """Monitor Arduino responses"""
        while self.arduino and self.arduino.is_open:
            try:
                if self.arduino.in_waiting > 0:
                    response = self.arduino.readline().decode().strip()
                    if response:
                        self.get_logger().info(f"🤖 Arduino: {response}")
                        
                        # Publish response
                        msg = String()
                        msg.data = response
                        self.response_publisher.publish(msg)
                        
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Monitor error: {e}")
                break
    
    def command_callback(self, msg):
        """Receive command and send to Arduino"""
        command = msg.data.strip()
        self.get_logger().info(f"📤 Sending: {command}")
        
        if self.arduino and self.arduino.is_open:
            try:
                # Create Arduino message format with VERSION field
                version = "1.0"  # Add this
                msg_id = int(time.time()) % 10000  # Shorter ID
                base_msg = f"MSG_START|{version}|{msg_id}|{command}"  # Include version
                checksum = sum(ord(c) for c in base_msg) % 256
                full_msg = f"{base_msg}|#{checksum}|MSG_END"
                
                # Send to Arduino
                self.arduino.write((full_msg + '\n').encode())
                self.arduino.flush()
                
                self.get_logger().info(f"📡 Sent: {full_msg}")
                
            except Exception as e:
                self.get_logger().error(f"❌ Send failed: {e}")
        else:
                self.get_logger().error("❌ Arduino not connected")

def main():
    rclpy.init()
    node = SimpleArduinoController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        if node.arduino and node.arduino.is_open:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
