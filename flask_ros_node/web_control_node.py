#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import threading
import time
import os
from ament_index_python.packages import get_package_share_directory

# Initialize Flask app with proper template/static paths
package_share_directory = get_package_share_directory('flask_ros_node')
template_folder = os.path.join(package_share_directory, 'templates')
static_folder = os.path.join(package_share_directory, 'static')

app = Flask(__name__, 
            template_folder=template_folder,
            static_folder=static_folder)
socketio = SocketIO(app, cors_allowed_origins="*")

class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_control_node')
        
        # ROS2 Publishers
        self.cmd_publisher = self.create_publisher(String, 'arduino_commands', 10)
        
        # ROS2 Subscribers (for Arduino feedback)
        self.feedback_subscriber = self.create_subscription(
            String,
            'arduino_feedback',
            self.feedback_callback,
            10
        )
        
        # State
        self.connected = True  # ROS2 is connected
        self.data_lock = threading.Lock()
        
        self.get_logger().info('Web Control Node started')
        
        # Emit initial status to web clients
        socketio.start_background_task(self.emit_status_periodically)
    
    def publish_command(self, command):
        """Publish command to ROS2 topic"""
        msg = String()
        msg.data = command
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f'Published command: {command}')
    
    def feedback_callback(self, msg):
        """Handle feedback from Arduino (via ROS2 topic)"""
        self.get_logger().info(f'Received feedback: {msg.data}')
        # Broadcast to web clients
        socketio.emit('arduino_message', {
            'message': msg.data,
            'timestamp': time.strftime('%H:%M:%S')
        })
    
    def emit_status_periodically(self):
        """Periodically emit connection status"""
        while rclpy.ok():
            socketio.emit('status', {'connected': self.connected})
            socketio.sleep(2)

# Global node instance
node = None

# Flask Routes
@app.route('/')
def index():
    return render_template('controller.html')

@app.route('/styles.css')
def styles():
    return app.send_static_file('css/styles.css'), 200, {'Content-Type': 'text/css'}

@app.route('/controller.js')
def controller_js():
    return app.send_static_file('js/controller.js'), 200, {'Content-Type': 'application/javascript'}

# SocketIO Events
@socketio.on('connect')
def handle_connect():
    print('Web client connected')
    emit('status', {'connected': node.connected if node else False})

@socketio.on('send_message')
def handle_send_message(data):
    """Receive motor command from frontend and publish to ROS2"""
    message = data.get('message', '').strip()
    if message and node:
        try:
            print(f"Sending to ROS2: {message}")
            node.publish_command(message)
            emit('message_sent', {
                'message': message,
                'timestamp': time.strftime('%H:%M:%S')
            })
        except Exception as e:
            print(f"Send error: {e}")
            emit('error', {'message': f'Error sending message: {e}'})
    else:
        emit('error', {'message': 'Node not ready or empty message'})

@socketio.on('disconnect')
def handle_disconnect():
    print('Web client disconnected')

def run_flask():
    """Run Flask-SocketIO server"""
    print("Starting Flask server on http://0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main(args=None):
    global node
    
    rclpy.init(args=args)
    node = WebControlNode()
    
    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()