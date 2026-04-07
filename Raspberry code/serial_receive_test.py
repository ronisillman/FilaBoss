import serial

ser = serial.Serial("/dev/serial0", 115200)

while True:
    line = ser.readline().decode().strip()
    if line:
        print(line)