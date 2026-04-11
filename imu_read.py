import serial
import time

# Make sure this matches your actual port and baud rate
arduino_port = '/dev/ttyACM0'
baud_rate = 9600

try:
    # Set timeout to 1 so the script doesn't hang forever if disconnected
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    ser.flush()
    print(f"Listening on {arduino_port} at {baud_rate} baud...")

    while True:
        if ser.in_waiting > 0:
            # Read the line, decode the bytes to a string, and strip the newline characters
            line = ser.readline().decode('utf-8').rstrip()
            print(line)

            # You can easily add if-statements here to parse specific lines
            # e.g., if line.startswith("YPR:"):

except KeyboardInterrupt:
    print("\nClosing serial connection.")
    ser.close()
except serial.SerialException as e:
    print(f"Error: {e}")
