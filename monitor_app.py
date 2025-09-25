from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import serial
import serial.tools.list_ports
import time
import threading

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
arduino = None
arduino_thread = None
running = False

def find_arduino_port():
    """Find the Arduino serial port"""
    ports = serial.tools.list_ports.comports()
    print("Available ports:")
    for port in ports:
        print(f"  {port.device} - {port.description}")
    
    # Look for Arduino (check for usbmodem on Mac)
    for port in ports:
        if ('Arduino' in port.description or 
            'CH340' in port.description or 
            'CP2102' in port.description or
            'usbmodem' in port.device):
            print(f"Found Arduino on: {port.device}")
            return port.device
    
    # If no Arduino found, use first available port
    if ports:
        print(f"No Arduino auto-detected. Using first port: {ports[0].device}")
        return ports[0].device
    
    return None

def connect_arduino():
    """Connect to Arduino via serial"""
    global arduino
    try:
        port = find_arduino_port()
        if port:
            print(f"Connecting to {port} at 9600 baud...")
            arduino = serial.Serial(port, 9600, timeout=1)
            time.sleep(3)  # Give Arduino time to reset
            print("Connected! Starting to monitor Arduino...")
            return True
        else:
            print("No Arduino port found")
            return False
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        return False

def arduino_monitor():
    """Monitor Arduino messages and send to web clients"""
    global arduino, running
    
    while running and arduino and arduino.is_open:
        try:
            if arduino.in_waiting:
                message = arduino.readline().decode().strip()
                if message:
                    print(f"Arduino says: {message}")
                    # Send message to all connected web clients
                    socketio.emit('arduino_message', {
                        'message': message,
                        'timestamp': time.strftime('%H:%M:%S')
                    })
            time.sleep(0.1)
        except Exception as e:
            print(f"Error reading from Arduino: {e}")
            # Try to reconnect
            print("Attempting to reconnect to Arduino...")
            if connect_arduino():
                print("Reconnected successfully!")
                continue
            else:
                print("Reconnection failed. Stopping monitor.")
                break

@app.route('/')
def index():
    return render_template('monitor.html')

@socketio.on('connect')
def handle_connect():
    print('Web client connected')
    emit('status', {'connected': arduino and arduino.is_open})

@socketio.on('disconnect')
def handle_disconnect():
    print('Web client disconnected')

if __name__ == '__main__':
    print("Starting Arduino Monitor Web App...")
    
    # Connect to Arduino
    if connect_arduino():
        running = True
        # Start Arduino monitoring thread
        arduino_thread = threading.Thread(target=arduino_monitor, daemon=True)
        arduino_thread.start()
        print("Arduino monitoring started!")
    else:
        print("Starting without Arduino connection.")
    
    print("Web server starting on http://localhost:5000")
    socketio.run(app, debug=False, host='0.0.0.0', port=5000)  # Turn off debug mode