import serial
import time

PORT = "/dev/ttyUSB0"  # Or /dev/serial0 for GPIO UART
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)

counter = 1

try:
    while True:
        # Send message
        msg = f"Hello Pi {counter}"
        ser.write((msg + "\n").encode())
        counter += 1

        # Read incoming messages and print them
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print("Received:", line)

        time.sleep(1)

except KeyboardInterrupt:
    print("Stopped by user")
finally:
    ser.close()