from __future__ import annotations

# pyright: reportMissingImports=false

from dataclasses import dataclass


@dataclass
class InputEvent:
    kind: str  # up | down | select | quit


class RotaryButtonInput:
    """Rotary encoder + push button reader for Raspberry Pi.

    Default pins are examples for BCM numbering. Adjust to your wiring.
    """

    def __init__(self, pin_clk: int = 17, pin_dt: int = 27, pin_sw: int = 22) -> None:
        try:
            from gpiozero import Button, RotaryEncoder
        except ImportError as exc:
            raise RuntimeError(
                "gpiozero is not installed. Run: pip install gpiozero"
            ) from exc

        self._encoder = RotaryEncoder(a=pin_clk, b=pin_dt, wrap=False, max_steps=0)
        self._button = Button(pin_sw, pull_up=True, bounce_time=0.03)
        self._last_steps = self._encoder.steps
        self._was_pressed = False

    def poll_events(self) -> list[InputEvent]:
        events: list[InputEvent] = []

        current_steps = self._encoder.steps
        delta = current_steps - self._last_steps
        if delta != 0:
            if delta > 0:
                events.append(InputEvent("down"))
            else:
                events.append(InputEvent("up"))
            self._last_steps = current_steps

        is_pressed = self._button.is_pressed
        if is_pressed and not self._was_pressed:
            events.append(InputEvent("select"))
        self._was_pressed = is_pressed

        return events

    def close(self) -> None:
        self._encoder.close()
        self._button.close()
