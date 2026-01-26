#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tennis_ball_msgs.msg import MotorCommand
import serial
import time
import threading
import os

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ArduinoControllerNode(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        
        # Arduino connection
        self.arduino = None
        self.arduino_lock = threading.Lock()
        self.connect_arduino()

        fast_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Faster than RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # No buffering
        )
        
        # Subscribe to MotorCommand messages
        self.motor_command_subscriber = self.create_subscription(
            MotorCommand,
            '/motor/command',
            self.motor_command_callback,
            fast_qos
        )
        
        # Publish Arduino responses
        self.response_publisher = self.create_publisher(
            String,
            '/arduino/response',
            10
        )
        
        self.get_logger().info('Arduino Controller Node started')
        self.get_logger().info('Listening for MotorCommand messages on: /motor/command')
        self.get_logger().info('Publishing Arduino responses on: /arduino/response')
    
    def connect_arduino(self):
        """Connect to Arduino"""
        if os.path.exists('/dev/ttyACM0'):
            try:
                self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=2)
                time.sleep(3)  # Wait for Arduino reset
                self.arduino.reset_input_buffer()
                self.get_logger().info("✅ Arduino connected on /dev/ttyACM0")
                
                # Start response monitor thread
                threading.Thread(target=self.monitor_arduino, daemon=True).start()
                
            except Exception as e:
                self.get_logger().error(f"❌ Arduino connection failed: {e}")
        else:
            self.get_logger().error("❌ /dev/ttyACM0 not found")
    
    def monitor_arduino(self):
        """Monitor Arduino responses"""
        self.get_logger().info("Arduino monitor thread started")
        while self.arduino and self.arduino.is_open:
            try:
                if self.arduino.in_waiting > 0:
                    response = self.arduino.readline().decode().strip()
                    if response:
                        self.get_logger().info(f"🤖 Arduino: {response}")
                        
                        # Publish response to ROS2
                        msg = String()
                        msg.data = response
                        self.response_publisher.publish(msg)
                        
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Monitor error: {e}")
                break
        self.get_logger().info("Arduino monitor thread stopped")
    
    def motor_command_callback(self, msg):
        """Handle incoming MotorCommand messages"""
        self.get_logger().info(f"📥 Received: {msg.motor} {msg.command} {msg.value} {msg.direction}")
        
        try:
            # Convert MotorCommand to Arduino protocol string
            arduino_cmd = self.convert_motor_command_to_arduino(msg)
            if arduino_cmd:
                self.send_arduino_command(arduino_cmd)
            else:
                self.get_logger().error(f"❌ Failed to convert command: {msg}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error handling motor command: {e}")
    
    def convert_motor_command_to_arduino(self, msg):
        """Convert MotorCommand to Arduino protocol string"""
        
        if msg.motor == "stepper":
            if msg.command == "move_steps":
                direction = msg.direction.upper() if msg.direction else "CW"
                return f"STEPPER=STEP_{direction}_{int(msg.value)}"
            elif msg.command == "move_angle":
                return f"STEPPER={int(msg.value)}"
            elif msg.command == "home":
                return "STEPPER=HOME"
            elif msg.command == "enable":
                return "STEPPER=ENABLE"
            elif msg.command == "disable":
                return "STEPPER=DISABLE"
            elif msg.command == "stop":
                return "STEPPER=STOP"
                
        elif msg.motor == "top_wheel":
            if msg.command == "set_rpm":
                pwm_percent = self.rpm_to_pwm(msg.value)
                return f"TOP_MOTOR={int(pwm_percent)}"
            elif msg.command == "set_percent":
                return f"TOP_MOTOR={int(msg.value)}"
            elif msg.command == "stop":
                return "TOP_MOTOR=0"
                
        elif msg.motor == "bottom_wheel":
            if msg.command == "set_rpm":
                pwm_percent = self.rpm_to_pwm(msg.value)
                return f"BOTTOM_MOTOR={int(pwm_percent)}"
            elif msg.command == "set_percent":
                return f"BOTTOM_MOTOR={int(msg.value)}"
            elif msg.command == "stop":
                return "BOTTOM_MOTOR=0"
                
        elif msg.motor == "linear_actuator":
            if msg.command == "set_angle":
                return f"LA={int(msg.value)}"
            elif msg.command == "set_percent":
                return f"LA={int(msg.value)}"
        
        # Unknown command
        self.get_logger().error(f"❌ Unknown motor command: {msg.motor} {msg.command}")
        return None
    
    def rpm_to_pwm(self, rpm):
        """Convert RPM to PWM percentage - adjust based on your motor specs"""
        max_rpm = 3000  # Adjust for your motors
        pwm_percent = min(100, max(0, (rpm / max_rpm) * 100))
        return pwm_percent
    
    def send_arduino_command(self, command):
        """Send command to Arduino with proper message format and thread safety"""
        with self.arduino_lock:
            if not (self.arduino and self.arduino.is_open):
                self.get_logger().error("❌ Arduino not connected")
                return False
                
            try:
                # Create Arduino message format: MSG_START|VERSION|MSG_ID|COMMAND|#CHECKSUM|MSG_END
                version = "1.0"
                msg_id = int(time.time()) % 10000  # Shorter ID
                base_message = f"MSG_START|{version}|{msg_id}|{command}"
                checksum = sum(ord(char) for char in base_message) % 256
                full_message = f"{base_message}|#{checksum}|MSG_END"
                
                # Send to Arduino
                self.arduino.write((full_message + '\n').encode())
                self.arduino.flush()
                
                self.get_logger().info(f"📡 Sent: {full_message}")
                return True
                
            except Exception as e:
                self.get_logger().error(f"❌ Send failed: {e}")
                return False

def main():
    rclpy.init()
    node = ArduinoControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Arduino Controller...")
    finally:
        if node.arduino and node.arduino.is_open:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
