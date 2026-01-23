import serial
import serial.tools.list_ports
import time

def find_arduino_port():
    """Find the Arduino serial port"""
    ports = serial.tools.list_ports.comports()
    print("Available ports:")
    for port in ports:
        print(f"  {port.device} - {port.description}")
    
    # Look for Arduino (check for usbmodem on Mac, common Arduino pattern)
    for port in ports:
        if ('Arduino' in port.description or 
            'CH340' in port.description or 
            'CP2102' in port.description or
            'usbmodem' in port.device):  # Mac Arduino pattern
            print(f"Found Arduino on: {port.device}")
            return port.device
    
    # If no Arduino found, let user choose
    if ports:
        print(f"No Arduino auto-detected. Using first port: {ports[0].device}")
        return ports[0].device
    
    return None

def main():
    print("Arduino Serial Test")
    print("=" * 30)
    
    # Find and connect to Arduino
    port = find_arduino_port()
    if not port:
        print("No serial ports found!")
        return
    
    try:
        print(f"Connecting to {port} at 9600 baud...")
        arduino = serial.Serial(port, 9600, timeout=1)
        time.sleep(3)  # Give Arduino time to reset
        print("Connected! Listening for Arduino messages...")
        print("Press Ctrl+C to exit")
        print("-" * 30)
        
        while True:
            if arduino.in_waiting:
                # Read message from Arduino
                message = arduino.readline().decode().strip()
                if message:
                    print(f"Arduino says: {message}")
            
            time.sleep(0.1)  # Small delay
            
    except serial.SerialException as e:
        print(f"Serial connection error: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'arduino' in locals():
            arduino.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    main()