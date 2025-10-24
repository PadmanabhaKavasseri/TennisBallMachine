from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import serial
import serial.tools.list_ports
import time
import threading

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Global state
arduino = None
arduino_thread = None
running = False

def find_arduino_port():
    """Scan for Arduino-compatible serial ports"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if ('Arduino' in port.description or 
            'CH340' in port.description or 
            'CP2102' in port.description or 
            'usbmodem' in port.device):
            return port.device
    return ports[0].device if ports else None

def connect_arduino():
    """Establish serial connection with Arduino"""
    global arduino
    try:
        port = find_arduino_port()
        if port:
            print(f"Connecting to Arduino on {port}")
            arduino = serial.Serial(port, 9600, timeout=2)
            time.sleep(3)  # Allow Arduino to reset
            arduino.reset_input_buffer()
            print("Arduino connected and ready")
            return True
        else:
            print("No Arduino found")
            return False
    except Exception as e:
        print(f"Connection error: {e}")
        return False

def arduino_monitor():
    """Continuously read from Arduino and broadcast to clients"""
    global arduino, running
    print("Arduino monitor thread started")
    while running and arduino and arduino.is_open:
        try:
            if arduino.in_waiting > 0:
                message = arduino.readline().decode().strip()
                if message:
                    print(f"Arduino says: {message}")
                    socketio.emit('arduino_message', {
                        'message': message,
                        'timestamp': time.strftime('%H:%M:%S')
                    })
            time.sleep(0.1)
        except Exception as e:
            print(f"Monitor error: {e}")
            break
    print("Arduino monitor thread stopped")

@app.route('/')
def index():
    return render_template('controller.html')

@app.route('/styles.css')
def styles():
    return render_template('styles.css'), 200, {'Content-Type': 'text/css'}

@app.route('/controller.js')
def controller_js():
    return render_template('controller.js'), 200, {'Content-Type': 'application/javascript'}

@socketio.on('connect')
def handle_connect():
    print("Web client connected")
    emit('status', {'connected': arduino and arduino.is_open})

@socketio.on('send_message')
def handle_send_message(data):
    """Receive motor command from frontend and send to Arduino"""
    message = data.get('message', '').strip()
    if message and arduino and arduino.is_open:
        try:
            print(f"Sending to Arduino: {message}")
            arduino.write((message + '\n').encode())
            arduino.flush()
            emit('message_sent', {
                'message': message,
                'timestamp': time.strftime('%H:%M:%S')
            })
        except Exception as e:
            print(f"Send error: {e}")
            emit('error', {'message': f'Error sending message: {e}'})
    else:
        emit('error', {'message': 'Arduino not connected or empty message'})

@socketio.on('disconnect')
def handle_disconnect():
    print("Web client disconnected")

if __name__ == '__main__':
    print("Starting Tennis Ball Machine Controller...")
    if connect_arduino():
        running = True
        arduino_thread = threading.Thread(target=arduino_monitor, daemon=True)
        arduino_thread.start()
    else:
        print("Running without Arduino connection")
    socketio.run(app, debug=False, host='0.0.0.0', port=5000)
