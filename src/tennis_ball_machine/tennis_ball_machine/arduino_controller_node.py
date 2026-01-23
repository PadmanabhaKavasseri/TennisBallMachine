#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from std_msgs.msg import String
import serial
import serial.tools.list_ports
import threading
import time
import platform
import os

class ArduinoControllerNode(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        
        # Arduino connection
        self.arduino = None
        self.arduino_thread = None
        self.running = False
        self.arduino_lock = threading.Lock()
        
        # System state
        self.current_mode = "manual"
        self.stepper_position = 0
        self.stepper_enabled = False
        
        # Connect to Arduino
        if self.connect_arduino():
            self.running = True
            self.arduino_thread = threading.Thread(target=self.arduino_monitor, daemon=True)
            self.arduino_thread.start()
            
            # Send home command after startup delay
            self.create_timer(3.0, self.send_home_command_once)
        
        # ROS2 subscribers
        self.motor_command_subscriber = self.create_subscription(
            String,
            '/motor/command',
            self.motor_command_callback,
            10
        )
        
        self.mode_subscriber = self.create_subscription(
            String,
            '/system/mode',
            self.mode_callback,
            10
        )
        
        # ROS2 publishers
        self.arduino_response_publisher = self.create_publisher(
            String,
            '/arduino/response',
            10
        )
        
        self.system_state_publisher = self.create_publisher(
            String,
            '/system/state',
            10
        )
        
        self.get_logger().info('Arduino Controller Node started')
    
    def send_home_command_once(self):
        """Send home command once on startup"""
        self.get_logger().info('Sending HOME command to Arduino...')
        self.send_arduino_command("STEPPER=HOME")
        # Cancel timer so it only runs once
        
    def connect_arduino(self):
        """Connect to Arduino using existing logic"""
        try:
            port = self.find_arduino_port()
            if port:
                self.get_logger().info(f"Connecting to Arduino on {port}")
                self.arduino = serial.Serial(port, 9600, timeout=2)
                time.sleep(3)  # Allow Arduino to reset
                self.arduino.reset_input_buffer()
                self.get_logger().info("Arduino connected and ready")
                return True
            else:
                self.get_logger().error("No Arduino found")
                return False
        except Exception as e:
            self.get_logger().error(f"Connection error: {e}")
            return False
    
    def find_arduino_port(self):
        """Find Arduino port (Linux-specific for your setup)"""
        if os.path.exists('/dev/ttyACM0'):
            try:
                # Test if we can actually open it
                test_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
                test_serial.close()
                return '/dev/ttyACM0'
            except serial.SerialException as e:
                if "Permission denied" in str(e):
                    self.get_logger().error("Permission denied. Add user to dialout group:")
                    self.get_logger().error("sudo usermod -a -G dialout $USER")
                return None
        return None
    
    def arduino_monitor(self):
        """Monitor Arduino responses and update system state"""
        self.get_logger().info("Arduino monitor thread started")
        while self.running and self.arduino and self.arduino.is_open:
            try:
                if self.arduino.in_waiting > 0:
                    message = self.arduino.readline().decode().strip()
                    if message:
                        self.get_logger().info(f"Arduino says: {message}")
                        
                        # Parse Arduino responses for state updates
                        self.parse_arduino_response(message)
                        
                        # Publish Arduino response to ROS2
                        response_msg = String()
                        response_msg.data = message
                        self.arduino_response_publisher.publish(response_msg)
                        
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Monitor error: {e}")
                break
        self.get_logger().info("Arduino monitor thread stopped")
    
    def parse_arduino_response(self, message):
        """Parse Arduino responses to update system state"""
        try:
            # Parse stepper position updates
            if "Position now:" in message:
                # Extract position from "Position now: 1234"
                parts = message.split("Position now:")
                if len(parts) > 1:
                    self.stepper_position = int(parts[1].strip())
                    self.publish_system_state()
            
            # Parse stepper enable/disable
            elif "Stepper enabled" in message:
                self.stepper_enabled = True
                self.publish_system_state()
            elif "Stepper disabled" in message:
                self.stepper_enabled = False
                self.publish_system_state()
            
            # Parse homing complete
            elif "Homing complete" in message:
                self.stepper_position = 0  # Reset position after homing
                self.stepper_enabled = True
                self.publish_system_state()
                
        except Exception as e:
            self.get_logger().error(f"Error parsing Arduino response: {e}")
    
    def publish_system_state(self):
        """Publish current system state"""
        state_msg = String()
        state_data = {
            'mode': self.current_mode,
            'stepper_position': self.stepper_position,
            'stepper_enabled': self.stepper_enabled,
            'timestamp': time.time()
        }
        state_msg.data = str(state_data)
        self.system_state_publisher.publish(state_msg)
    
    def motor_command_callback(self, msg):
        """Handle incoming motor commands as strings"""
        try:
            # Parse string like "stepper move_steps 100 cw"
            parts = msg.data.split()
            if len(parts) >= 2:
                motor = parts[0]
                command = parts[1]
                value = float(parts[2]) if len(parts) > 2 else 0
                direction = parts[3] if len(parts) > 3 else ""
                
                self.get_logger().info(f"Received command: {motor} {command} {value} {direction}")
                
                # Create a simple command object
                class SimpleCommand:
                    def __init__(self, motor, command, value, direction):
                        self.motor = motor
                        self.command = command
                        self.value = value
                        self.direction = direction
                        self.speed = 0.0
                
                cmd = SimpleCommand(motor, command, value, direction)
                arduino_cmd = self.convert_motor_command_to_arduino(cmd)
                if arduino_cmd:
                    self.send_arduino_command(arduino_cmd)
                    
        except Exception as e:
            self.get_logger().error(f"Error parsing command: {e}")
    
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
        
        return None
    
    def rpm_to_pwm(self, rpm):
        """Convert RPM to PWM percentage - adjust based on your motor specs"""
        # Example conversion - adjust for your actual motors
        max_rpm = 3000  # Adjust based on your motor specifications
        pwm_percent = min(100, max(0, (rpm / max_rpm) * 100))
        return pwm_percent
    
    def mode_callback(self, msg):
        """Handle system mode changes"""
        self.current_mode = msg.data
        self.get_logger().info(f"System mode changed to: {self.current_mode}")
        self.publish_system_state()
    
    def send_arduino_command(self, command):
        """Send command to Arduino with proper message format and thread safety"""
        with self.arduino_lock:
            if not (self.arduino and self.arduino.is_open):
                self.get_logger().error("Arduino not connected")
                return False
                
            try:
                # Create proper Arduino message format
                msg_id = int(time.time() * 1000)
                base_message = f"MSG_START|{msg_id}|{command}"
                checksum = sum(ord(char) for char in base_message) % 256
                full_message = f"{base_message}|#{checksum}|MSG_END"
                
                # Send to Arduino
                self.arduino.write((full_message + '\n').encode())
                self.arduino.flush()
                
                self.get_logger().info(f"Sent to Arduino: {full_message}")
                return True
                
            except Exception as e:
                self.get_logger().error(f"Send error: {e}")
                return False

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        if node.arduino and node.arduino.is_open:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
