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
    """Find the Arduino serial port automatically"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Look for common Arduino identifiers including Mac's usbmodem pattern
        if ('Arduino' in port.description or 
            'CH340' in port.description or 
            'CP2102' in port.description or
            'usbmodem' in port.device):  # Mac Arduino pattern
            return port.device
    # If no Arduino found, return the first available port
    if ports:
        return ports[0].device
    return None

def connect_arduino():
    """Connect to Arduino via serial"""
    global arduino
    try:
        port = find_arduino_port()
        if port:
            print(f"Attempting to connect to Arduino on {port}")
            arduino = serial.Serial(port, 9600, timeout=2)  # Longer timeout
            time.sleep(3)  # Give Arduino time to reset and start
            print(f"Connected to Arduino on {port}")
            
            # Clear any startup messages
            time.sleep(0.5)
            arduino.reset_input_buffer()
            
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
    
    print("DEBUG: Arduino monitor thread started")
    
    while running and arduino and arduino.is_open:
        try:
            bytes_waiting = arduino.in_waiting
            if bytes_waiting > 0:
                print(f"DEBUG: {bytes_waiting} bytes waiting from Arduino")
                message = arduino.readline().decode().strip()
                if message:
                    print(f"DEBUG: Arduino says: '{message}'")
                    # Send message to all connected web clients
                    socketio.emit('arduino_message', {
                        'message': message,
                        'timestamp': time.strftime('%H:%M:%S')
                    })
                else:
                    print("DEBUG: Empty message received from Arduino")
            time.sleep(0.1)
        except Exception as e:
            print(f"DEBUG: Error reading from Arduino: {e}")
            # Try to reconnect
            print("DEBUG: Attempting to reconnect to Arduino...")
            if connect_arduino():
                print("DEBUG: Reconnected successfully!")
                continue
            else:
                print("DEBUG: Reconnection failed. Stopping monitor.")
                break
    
    print("DEBUG: Arduino monitor thread ended")

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    print('Web client connected')
    emit('status', {'connected': arduino and arduino.is_open})

@socketio.on('send_message')
def handle_send_message(data):
    """Handle message from web client to send to Arduino"""
    message = data.get('message', '').strip()
    if message and arduino and arduino.is_open:
        try:
            print(f"DEBUG: Sending to Arduino: '{message}'")
            arduino.write((message + '\n').encode())
            arduino.flush()
            
            # Confirm message was sent to the web interface
            emit('message_sent', {
                'message': message,
                'timestamp': time.strftime('%H:%M:%S')
            })
            
            # The Arduino response will be captured by the arduino_monitor() function
            # and sent to the web via 'arduino_message' event
            
        except Exception as e:
            print(f"ERROR: Error sending to Arduino: {e}")
            emit('error', {'message': f'Error sending message: {e}'})
    else:
        print("ERROR: Arduino not connected or empty message")
        emit('error', {'message': 'Arduino not connected or empty message'})

@socketio.on('disconnect')
def handle_disconnect():
    print('Web client disconnected')

if __name__ == '__main__':
    print("Starting Arduino Web Controller...")
    
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
    socketio.run(app, debug=False, host='0.0.0.0', port=5000)