import serial
import time

ser = serial.Serial("/dev/ttyUSB9", 115200)

count = 1

while True:
    msg = f"Hello ESP32 {count}\n"
    ser.write(msg.encode())
    print("TX:", msg.strip())

    count += 1
    time.sleep(1)