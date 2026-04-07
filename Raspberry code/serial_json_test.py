import argparse
import json
import time

import serial


DEFAULT_COMMANDS = {
    "pid_p_pulley": 5000.0,
    "pid_i_pulley": 1500.0,
    "pid_d_pulley": 5.0,
    "pid_p_spool": 7000.0,
    "pid_i_spool": 2500.0,
    "pid_d_spool": 5.0,
    "fan_speed_pct": 100,
    "target_mode": "Spd",
    "target_diameter_mm": 1.75,
    "target_speed_mps": 0.01,
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Test the ESP32 Raspberry JSON serial link.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port for the ESP32")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    commands = dict(DEFAULT_COMMANDS)

    counter = 0

    try:
        while True:
            commands["fan_speed_pct"] = (counter * 10) % 101
            commands["target_mode"] = "Spd" if counter % 2 == 0 else "Dia"
            commands["target_speed_mps"] = 0.01 + (counter % 5) * 0.002
            commands["target_diameter_mm"] = 1.75 + (counter % 3) * 0.05

            payload = json.dumps(commands)
            ser.write((payload + "\n").encode("utf-8"))
            print("TX:", payload)

            deadline = time.time() + 0.75
            while time.time() < deadline:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                try:
                    message = json.loads(line)
                except json.JSONDecodeError:
                    print("RX:", line)
                else:
                    print("RX:", json.dumps(message, indent=2))

            counter += 1
            time.sleep(0.25)

    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        ser.close()


if __name__ == "__main__":
    main()