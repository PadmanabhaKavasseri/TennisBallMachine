#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tennis_ball_msgs.msg import MotorCommand
from std_msgs.msg import String
import time
import threading

class ModeManagerNode(Node):
    def __init__(self):
        super().__init__('mode_manager')
        
        # System state
        self.current_mode = "manual"  # "manual" or "auto"
        self.mode_lock = threading.Lock()
        self.last_command_time = 0
        self.command_timeout = 5.0  # 5 seconds timeout for auto mode
        
        # Publishers
        self.motor_command_publisher = self.create_publisher(
            MotorCommand,
            '/motor/command',
            10
        )
        
        self.system_status_publisher = self.create_publisher(
            String,
            '/system/status',
            10
        )
        
        # Subscribers
        # Web interface commands
        self.web_command_subscriber = self.create_subscription(
            MotorCommand,
            '/web/motor_command',
            self.web_command_callback,
            10
        )
        
        # Person tracker commands
        self.auto_command_subscriber = self.create_subscription(
            MotorCommand,
            '/auto/motor_command',
            self.auto_command_callback,
            10
        )
        
        # Mode change requests
        self.mode_change_subscriber = self.create_subscription(
            String,
            '/system/set_mode',
            self.mode_change_callback,
            10
        )
        
        # Emergency stop
        self.emergency_stop_subscriber = self.create_subscription(
            String,
            '/system/emergency_stop',
            self.emergency_stop_callback,
            10
        )
        
        # Status publishing timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Mode Manager Node started')
        self.get_logger().info(f'Initial mode: {self.current_mode}')
        self.publish_status()
    
    def web_command_callback(self, msg):
        """Handle commands from web interface"""
        with self.mode_lock:
            if self.current_mode == "manual":
                self.get_logger().info(f'🌐 Web command (MANUAL): {msg.motor} {msg.command} {msg.value}')
                self.forward_command(msg, "web")
            else:
                self.get_logger().warn(f'🚫 Web command blocked (AUTO mode): {msg.motor} {msg.command}')
                # Could emit a warning back to web interface here
    
    def auto_command_callback(self, msg):
        """Handle commands from person tracker (auto mode)"""
        with self.mode_lock:
            if self.current_mode == "auto":
                self.get_logger().info(f'🤖 Auto command (AUTO): {msg.motor} {msg.command} {msg.value}')
                self.forward_command(msg, "auto")
                self.last_command_time = time.time()
            else:
                self.get_logger().info(f'🚫 Auto command blocked (MANUAL mode): {msg.motor} {msg.command}')
    
    def mode_change_callback(self, msg):
        """Handle mode change requests"""
        new_mode = msg.data.strip().lower()
        
        if new_mode in ["manual", "auto"]:
            with self.mode_lock:
                old_mode = self.current_mode
                self.current_mode = new_mode
                
                self.get_logger().info(f'🔄 Mode changed: {old_mode} → {new_mode}')
                
                # Send stop command when switching to manual mode
                if new_mode == "manual" and old_mode == "auto":
                    self.send_stop_command("Mode switch to manual")
                
                self.publish_status()
        else:
            self.get_logger().error(f'❌ Invalid mode: {new_mode}. Use "manual" or "auto"')
    
    def emergency_stop_callback(self, msg):
        """Handle emergency stop - always works regardless of mode"""
        with self.mode_lock:
            self.get_logger().warn(f'🛑 EMERGENCY STOP: {msg.data}')
            
            # Switch to manual mode
            old_mode = self.current_mode
            self.current_mode = "manual"
            
            # Send stop command to all motors
            self.send_stop_command("Emergency stop")
            
            self.get_logger().warn(f'🛑 Emergency stop complete. Mode: {old_mode} → manual')
            self.publish_status()
    
    def forward_command(self, msg, source):
        """Forward command to Arduino controller"""
        try:
            # Add source information for debugging
            self.get_logger().info(f'📤 Forwarding to Arduino ({source}): {msg.motor} {msg.command}')
            
            # Forward the command unchanged
            self.motor_command_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to forward command: {e}')
    
    def send_stop_command(self, reason):
        """Send stop command to stepper motor"""
        try:
            stop_cmd = MotorCommand()
            stop_cmd.motor = "stepper"
            stop_cmd.command = "stop"
            stop_cmd.value = 0.0
            stop_cmd.direction = ""
            stop_cmd.speed = 0.0
            
            self.motor_command_publisher.publish(stop_cmd)
            self.get_logger().info(f'🛑 Stop command sent: {reason}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Failed to send stop command: {e}')
    
    def publish_status(self):
        """Publish current system status"""
        try:
            status_data = {
                'mode': self.current_mode,
                'timestamp': time.time(),
                'last_auto_command': self.last_command_time,
                'auto_timeout': self.command_timeout
            }
            
            status_msg = String()
            status_msg.data = str(status_data)
            self.system_status_publisher.publish(status_msg)
            
            # Log status periodically
            if int(time.time()) % 10 == 0:  # Every 10 seconds
                self.get_logger().info(f'📊 Status: Mode={self.current_mode}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Failed to publish status: {e}')
    
    def check_auto_timeout(self):
        """Check if auto mode has timed out (no recent commands)"""
        if self.current_mode == "auto":
            time_since_last = time.time() - self.last_command_time
            if time_since_last > self.command_timeout:
                self.get_logger().warn(f'⏰ Auto mode timeout ({time_since_last:.1f}s)')
                # Could switch to manual mode or send alert

def main():
    rclpy.init()
    node = ModeManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Mode Manager...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
