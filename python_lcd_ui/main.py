from __future__ import annotations

# pyright: reportMissingImports=false

import argparse
import importlib
import json
import time
from dataclasses import asdict, dataclass
from typing import Any

from display_hardware import HardwareLcdDisplay
from display_simulator import SimulatedLcdDisplay
from input_devices import InputEvent, RotaryButtonInput
from ui_controller import UiController


@dataclass
class TelemetryFromEsp32:
    load_mode: bool
    filament_speed_mps: float
    fan_rpm: float
    diameter_travelled_mm: float

    @classmethod
    def from_json_line(cls, line: str) -> TelemetryFromEsp32:
        data = json.loads(line)
        if not isinstance(data, dict):
            raise ValueError("Telemetry JSON must be an object")

        return cls(
            load_mode=bool(data.get("load_mode", False)),
            filament_speed_mps=float(data.get("filament_speed_mps", 0.0)),
            fan_rpm=float(data.get("fan_rpm", 0.0)),
            diameter_travelled_mm=float(data.get("diameter_travelled_mm", 0.0)),
        )

    def apply_to_controller(self, controller: UiController) -> None:
        state = controller.state
        state.load_mode = self.load_mode
        state.pulley_speed_mps = self.filament_speed_mps
        state.fan_rpm = self.fan_rpm


@dataclass
class CommandsToEsp32:
    pid_p_pulley: int
    pid_i_pulley: int
    pid_d_pulley: int
    pid_p_spool: int
    pid_i_spool: int
    pid_d_spool: int
    fan_speed_pct: int
    target_mode: str
    target_diameter_mm: float
    target_speed_mps: float

    @classmethod
    def from_controller(cls, controller: UiController) -> CommandsToEsp32:
        state = controller.state
        return cls(
            pid_p_pulley=state.pulley_gains.p.digits,
            pid_i_pulley=state.pulley_gains.i.digits,
            pid_d_pulley=state.pulley_gains.d.digits,
            pid_p_spool=state.spool_gains.p.digits,
            pid_i_spool=state.spool_gains.i.digits,
            pid_d_spool=state.spool_gains.d.digits,
            fan_speed_pct=state.fan_speed_pct,
            target_mode=state.target_mode,
            target_diameter_mm=state.target_diameter_hundredths / 100.0,
            target_speed_mps=state.target_speed_tenths / 10.0,
        )

    def to_json_line(self) -> str:
        payload: dict[str, Any] = asdict(self)
        return json.dumps(payload, separators=(",", ":")) + "\n"


class SerialJsonBridge:
    """Newline-delimited JSON bridge for ESP32 communication."""

    def __init__(self, port: str, baudrate: int, timeout: float = 0.0) -> None:
        try:
            serial = importlib.import_module("serial")
        except ImportError as exc:
            raise RuntimeError("pyserial is required for serial communication. Run: pip install pyserial") from exc

        self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self._rx_buffer = ""

    def read_line(self) -> str | None:
        waiting = int(self._serial.in_waiting)
        if waiting <= 0:
            return None

        chunk = self._serial.read(waiting).decode("utf-8", errors="replace")
        if not chunk:
            return None

        self._rx_buffer += chunk
        if "\n" not in self._rx_buffer:
            return None

        line, self._rx_buffer = self._rx_buffer.split("\n", 1)
        return line.strip()

    def write_line(self, line: str) -> None:
        self._serial.write(line.encode("utf-8"))

    def close(self) -> None:
        self._serial.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="FilaBoss LCD UI (simulator and Raspberry Pi hardware modes)"
    )
    parser.add_argument("--mode", choices=["sim", "hw"], default="sim")
    parser.add_argument("--cols", type=int, default=20)
    parser.add_argument("--rows", type=int, default=4)

    parser.add_argument("--i2c-address", type=lambda x: int(x, 0), default=0x27)
    parser.add_argument("--i2c-port", type=int, default=1)

    parser.add_argument("--pin-clk", type=int, default=17)
    parser.add_argument("--pin-dt", type=int, default=27)
    parser.add_argument("--pin-sw", type=int, default=22)

    parser.add_argument("--serial-port", type=str, default="")
    parser.add_argument("--serial-baudrate", type=int, default=115200)
    parser.add_argument("--serial-tx-hz", type=float, default=20.0)

    parser.add_argument("--fps", type=float, default=12.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    controller = UiController(cols=args.cols, rows=args.rows)

    display = None
    input_device = None
    serial_bridge: SerialJsonBridge | None = None

    try:
        if args.mode == "sim":
            display = SimulatedLcdDisplay(cols=args.cols, rows=args.rows)
        else:
            display = HardwareLcdDisplay(
                cols=args.cols,
                rows=args.rows,
                i2c_address=args.i2c_address,
                i2c_port=args.i2c_port,
            )
            input_device = RotaryButtonInput(
                pin_clk=args.pin_clk,
                pin_dt=args.pin_dt,
                pin_sw=args.pin_sw,
            )
            if args.serial_port:
                serial_bridge = SerialJsonBridge(
                    port=args.serial_port,
                    baudrate=args.serial_baudrate,
                    timeout=0.0,
                )

        frame_delay = 1.0 / max(1.0, args.fps)
        tx_period = 1.0 / max(1.0, args.serial_tx_hz)
        next_tx_time = time.monotonic()
        running = True

        while running:
            events: list[InputEvent] = []
            if args.mode == "sim":
                display.refresh_ui()
                if not display.is_open:
                    break
                events = display.poll_events()
            elif input_device is not None:
                events = input_device.poll_events()

            if args.mode == "hw" and serial_bridge is not None:
                telemetry_line = serial_bridge.read_line()
                if telemetry_line:
                    try:
                        telemetry = TelemetryFromEsp32.from_json_line(telemetry_line)
                        telemetry.apply_to_controller(controller)
                    except (ValueError, json.JSONDecodeError):
                        # Ignore malformed telemetry lines and keep last valid state.
                        pass

            for event in events:
                if args.mode == "hw":
                    if event.kind == "up":
                        print("encoder left")
                    elif event.kind == "down":
                        print("encoder right")
                    elif event.kind == "select":
                        print("encoder pushed")
                if not controller.handle_event(event):
                    running = False
                    break

            controller.tick()

            if args.mode == "hw" and serial_bridge is not None:
                now = time.monotonic()
                if now >= next_tx_time:
                    commands_json = CommandsToEsp32.from_controller(controller).to_json_line()
                    serial_bridge.write_line(commands_json)
                    next_tx_time = now + tx_period

            lines = controller.render_lines()
            for row, text in enumerate(lines):
                display.write_line(row, text)

            time.sleep(frame_delay)

    except KeyboardInterrupt:
        pass
    finally:
        if serial_bridge is not None:
            serial_bridge.close()
        if input_device is not None:
            input_device.close()
        if display is not None:
            display.close()


if __name__ == "__main__":
    main()
