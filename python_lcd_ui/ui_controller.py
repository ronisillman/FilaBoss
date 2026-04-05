from __future__ import annotations

import math
import time
from dataclasses import dataclass

from input_devices import InputEvent


@dataclass
class AppState:
    page_index: int = 0
    edit_mode: bool = False
    target_temp_c: int = 205
    current_temp_c: float = 25.0
    motor_on: bool = False
    motor_speed_pct: int = 35


class UiController:
    """UI logic independent from display/input implementation."""

    pages = ["STATUS", "TEMP", "MOTOR"]

    def __init__(self, cols: int = 20, rows: int = 4) -> None:
        self.cols = cols
        self.rows = rows
        self.state = AppState()
        self._start = time.monotonic()

    def handle_event(self, event: InputEvent) -> bool:
        if event.kind == "quit":
            return False

        if event.kind == "select":
            self._handle_select()
            return True

        if event.kind in {"up", "down"}:
            self._handle_turn(event.kind)
            return True

        return True

    def _handle_select(self) -> None:
        if self.state.page_index == 0:
            self.state.motor_on = not self.state.motor_on
            return

        if self.state.page_index == 1:
            self.state.edit_mode = not self.state.edit_mode
            return

        if self.state.page_index == 2:
            self.state.edit_mode = not self.state.edit_mode

    def _handle_turn(self, direction: str) -> None:
        step = -1 if direction == "up" else 1

        if not self.state.edit_mode:
            self.state.page_index = (self.state.page_index + step) % len(self.pages)
            return

        if self.state.page_index == 1:
            self.state.target_temp_c = max(150, min(280, self.state.target_temp_c + step))
            return

        if self.state.page_index == 2:
            self.state.motor_speed_pct = max(0, min(100, self.state.motor_speed_pct + step))

    def tick(self) -> None:
        elapsed = time.monotonic() - self._start
        wobble = math.sin(elapsed * 0.9) * 1.6
        trend = (self.state.target_temp_c - self.state.current_temp_c) * 0.02
        self.state.current_temp_c += trend + wobble * 0.01

    def render_lines(self) -> list[str]:
        page = self.state.page_index
        if page == 0:
            lines = self._render_status_page()
        elif page == 1:
            lines = self._render_temp_page()
        else:
            lines = self._render_motor_page()

        rendered = [(line[: self.cols]).ljust(self.cols) for line in lines]
        if len(rendered) < self.rows:
            rendered.extend([" " * self.cols for _ in range(self.rows - len(rendered))])
        return rendered[: self.rows]

    def _render_status_page(self) -> list[str]:
        mode = "ON" if self.state.motor_on else "OFF"
        return [
            "FilaBoss    [STATUS]",
            f"Temp: {self.state.current_temp_c:5.1f}C",
            f"Motor: {mode:<3}  {self.state.motor_speed_pct:3d}%",
            "Sel=Motor  Turn=Page",
        ]

    def _render_temp_page(self) -> list[str]:
        mode = "EDIT" if self.state.edit_mode else "VIEW"
        return [
            "FilaBoss      [TEMP]",
            f"Target: {self.state.target_temp_c:3d}C",
            f"Now:    {self.state.current_temp_c:5.1f}C",
            f"Mode: {mode:<4} Sel=Toggle",
        ]

    def _render_motor_page(self) -> list[str]:
        mode = "EDIT" if self.state.edit_mode else "VIEW"
        state = "ON" if self.state.motor_on else "OFF"
        return [
            "FilaBoss     [MOTOR]",
            f"State: {state:<3} Sel=Edit ",
            f"Speed: {self.state.motor_speed_pct:3d}%",
            f"Mode: {mode:<4} Sel=Exit ",
        ]
