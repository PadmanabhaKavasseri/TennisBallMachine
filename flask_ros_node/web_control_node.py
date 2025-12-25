#!/home/ubuntu/ros2_ws/src/flask_ros_node/venv/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template, request, jsonify
import threading

app = Flask(__name__)

class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_control_node')
        
        # ROS2 publisher
        self.publisher_ = self.create_publisher(String, 'web_commands', 10)
        
        # Thread-safe storage for latest data
        self.latest_data = ""
        self.data_lock = threading.Lock()
        
        self.get_logger().info('Web Control Node has been started')
    
    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {command}')

# Global node instance
node = None

@app.route('/')
def index():
    return '''
    <html>
        <body>
            <h1>ROS2 Web Control</h1>
            <button onclick="sendCommand('forward')">Forward</button>
            <button onclick="sendCommand('backward')">Backward</button>
            <button onclick="sendCommand('stop')">Stop</button>
            
            <script>
                function sendCommand(cmd) {
                    fetch('/command', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify({command: cmd})
                    });
                }
            </script>
        </body>
    </html>
    '''

@app.route('/command', methods=['POST'])
def command():
    data = request.get_json()
    command = data.get('command', '')
    if node:
        node.publish_command(command)
    return jsonify({'status': 'success'})

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

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