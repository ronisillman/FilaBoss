from __future__ import annotations

import argparse
import time

from display_hardware import HardwareLcdDisplay
from display_simulator import SimulatedLcdDisplay
from input_devices import InputEvent, RotaryButtonInput
from ui_controller import UiController


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

    parser.add_argument("--fps", type=float, default=12.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    controller = UiController(cols=args.cols, rows=args.rows)

    display = None
    input_device = None

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

        frame_delay = 1.0 / max(1.0, args.fps)
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
            lines = controller.render_lines()
            for row, text in enumerate(lines):
                display.write_line(row, text)

            time.sleep(frame_delay)

    except KeyboardInterrupt:
        pass
    finally:
        if input_device is not None:
            input_device.close()
        if display is not None:
            display.close()


if __name__ == "__main__":
    main()
