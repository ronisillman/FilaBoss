import time
import serial

PORT = "/dev/serial0"   # Usually serial0 on Raspberry Pi
BAUD = 115200

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    print(f"Opened {PORT} at {BAUD}")

    last_tx = 0.0
    tx_count = 0

    try:
        while True:
            # Read anything from ESP32
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print("RX <-", line)

            # Send one test line every second
            now = time.monotonic()
            if now - last_tx >= 1.0:
                msg = f"TEST:{tx_count}"
                ser.write((msg + "\n").encode("utf-8"))
                print("TX ->", msg)

                tx_count += 1
                last_tx = now

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        ser.close()

if __name__ == "__main__":
    main()