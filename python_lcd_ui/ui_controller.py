from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from input_devices import InputEvent


@dataclass
class GainSetting:
    digits: int
    dot_index: int  # 0..5, decimal point insertion index from left


@dataclass
class PidGains:
    p: GainSetting = field(default_factory=lambda: GainSetting(12000, 2))
    i: GainSetting = field(default_factory=lambda: GainSetting(5000, 2))
    d: GainSetting = field(default_factory=lambda: GainSetting(800, 2))


@dataclass
class AppState:
    menu_index: int = 0
    focus_index: int = 0
    menu_edit: bool = False
    edit_target: str = ""
    gain_edit_step: int = 0
    filament_speed_mpm: float = 12.0
    filament_diameter_mm: float = 1.75
    fan_speed_pct: int = 35
    fan_rpm: float = 700.0
    pulley_gains: PidGains = field(default_factory=PidGains)
    spool_gains: PidGains = field(
        default_factory=lambda: PidGains(
            p=GainSetting(9000, 2),
            i=GainSetting(3500, 2),
            d=GainSetting(600, 2),
        )
    )
    pid_motor_index: int = 0


class UiController:
    """UI logic independent from display/input implementation."""

    menus = ["MAIN", "PID", "FAN"]

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
        if self.state.menu_edit:
            if self.state.edit_target in {"P", "I", "D"}:
                if self.state.gain_edit_step < 5:
                    self.state.gain_edit_step += 1
                    return

            self.state.menu_edit = False
            self.state.edit_target = ""
            self.state.gain_edit_step = 0
            return

        focus_item = self._current_focus_item()

        if focus_item in self.menus:
            self._switch_menu(focus_item)
            return

        if focus_item in {"MOTOR", "P", "I", "D", "FAN_SPEED"}:
            self.state.menu_edit = True
            self.state.edit_target = focus_item
            self.state.gain_edit_step = 0

    def _handle_turn(self, direction: str) -> None:
        step = -1 if direction == "up" else 1

        if self.state.menu_edit:
            self._apply_edit(step)
            return

        items = self._menu_items()
        self.state.focus_index = (self.state.focus_index + step) % len(items)

    def _apply_edit(self, step: int) -> None:
        target = self.state.edit_target

        if target == "FAN_SPEED":
            self.state.fan_speed_pct = max(0, min(100, self.state.fan_speed_pct + step * 5))
            return

        if target == "MOTOR":
            self.state.pid_motor_index = (self.state.pid_motor_index + step) % 2
            return

        if target in {"P", "I", "D"}:
            gain = self._gain_by_label(target)
            if self.state.gain_edit_step == 0:
                gain.dot_index = (gain.dot_index + step) % 6
                return

            self._adjust_gain_digit(gain, self.state.gain_edit_step - 1, step)

    def _switch_menu(self, menu_name: str) -> None:
        self.state.menu_index = self.menus.index(menu_name)
        # Set default focus target per menu.
        if menu_name == "PID":
            self.state.focus_index = 3  # MOTOR in PID items
        elif menu_name == "FAN":
            self.state.focus_index = 3  # FAN_SPEED in FAN items
        else:
            self.state.focus_index = self.menus.index(menu_name)
        self.state.menu_edit = False
        self.state.edit_target = ""
        self.state.gain_edit_step = 0

    def _menu_items(self) -> list[str]:
        if self.state.menu_index == 1:
            return ["MAIN", "PID", "FAN", "MOTOR", "P", "I", "D"]
        if self.state.menu_index == 2:
            return ["MAIN", "PID", "FAN", "FAN_SPEED"]
        return ["MAIN", "PID", "FAN"]

    def _current_focus_item(self) -> str:
        items = self._menu_items()
        if not items:
            return ""
        idx = max(0, min(self.state.focus_index, len(items) - 1))
        return items[idx]

    def tick(self) -> None:
        elapsed = time.monotonic() - self._start
        speed_wave = 10.0 + math.sin(elapsed * 0.8) * 0.8
        diameter_wave = 1.75 + math.sin(elapsed * 0.35) * 0.02
        fan_trim = (self.state.fan_speed_pct - 50) * 0.02
        self.state.filament_speed_mpm = speed_wave + fan_trim
        self.state.filament_diameter_mm = diameter_wave

        target_fan_rpm = self.state.fan_speed_pct * 20.0
        raw_offset = math.sin(elapsed * 0.55 + 0.6) * 20.0
        stepped_offset = round(raw_offset / 20.0) * 20.0
        stepped_offset = max(-20.0, min(20.0, stepped_offset))
        self.state.fan_rpm = max(0.0, min(2000.0, target_fan_rpm + stepped_offset))

    def _blink_on(self) -> bool:
        elapsed = time.monotonic() - self._start
        return int(elapsed * 3) % 2 == 0

    def render_lines(self) -> list[str]:
        if self.state.menu_index == 0:
            body = self._render_main_menu()
        elif self.state.menu_index == 1:
            body = self._render_pid_menu()
        else:
            body = self._render_fan_menu()

        lines = [self._render_tab_strip(), *body]

        rendered = [(line[: self.cols]).ljust(self.cols) for line in lines]
        if len(rendered) < self.rows:
            rendered.extend([" " * self.cols for _ in range(self.rows - len(rendered))])
        return rendered[: self.rows]

    def _render_tab_strip(self) -> str:
        focus_item = self._current_focus_item() if not self.state.menu_edit else ""
        blink_on = self._blink_on()
        tabs = [
            self._tab_label("MAIN", focus_item == "MAIN", blink_on),
            self._tab_label("PID", focus_item == "PID", blink_on),
            self._tab_label("FAN", focus_item == "FAN", blink_on),
        ]
        return f"{tabs[0]} {tabs[1]} {tabs[2]}"

    def _tab_label(self, label: str, focused: bool, blink_on: bool) -> str:
        if not focused:
            return label
        return self._focused_bracket(label, blink_on)

    def _render_main_menu(self) -> list[str]:
        return [
            f"Dia: {self.state.filament_diameter_mm:4.2f} mm",
            f"Spd: {self.state.filament_speed_mpm:4.1f} mm/s",
            f"Fan: {self._fan_speed_rpm():4d} rpm",
        ]

    def _render_pid_menu(self) -> list[str]:
        motor_name = self._current_motor_name()
        gains = self._current_gains()
        blink_on = self._blink_on()
        focus_item = self._current_focus_item() if not self.state.menu_edit else ""

        p_digit = self._active_digit_for_label("P")
        i_digit = self._active_digit_for_label("I")
        d_digit = self._active_digit_for_label("D")

        p_dot = self.state.menu_edit and self.state.edit_target == "P" and self.state.gain_edit_step == 0
        i_dot = self.state.menu_edit and self.state.edit_target == "I" and self.state.gain_edit_step == 0
        d_dot = self.state.menu_edit and self.state.edit_target == "D" and self.state.gain_edit_step == 0

        return [
            self._pid_motor_line(
                motor_name,
                focused=(focus_item == "MOTOR" and not self.state.menu_edit),
                editing=(self.state.menu_edit and self.state.edit_target == "MOTOR"),
                blink_on=blink_on,
            ),
            f"{self._pid_gain_segment('P', gains.p, p_digit, p_dot, blink_on, focused=(focus_item == 'P' and not self.state.menu_edit))} {self._pid_gain_segment('I', gains.i, i_digit, i_dot, blink_on, focused=(focus_item == 'I' and not self.state.menu_edit))}",
            self._pid_gain_segment("D", gains.d, d_digit, d_dot, blink_on, focused=(focus_item == "D" and not self.state.menu_edit)),
        ]
    
    def _render_fan_menu(self) -> list[str]:
        blink_on = self._blink_on()
        focused = self._current_focus_item() == "FAN_SPEED" and not self.state.menu_edit

        fan_text = f"{self.state.fan_speed_pct}%"
        width = len(fan_text)

        if self.state.menu_edit:
            inner = fan_text if blink_on else " " * width
            fan_text = f"[{inner}]"
        elif focused:
            fan_text = self._focused_bracket(fan_text, blink_on)
        else:
            fan_text = f" {fan_text}"

        rpm_value = self._fan_speed_rpm()
        rpm_width = len(str(rpm_value))
        rpm_text = f"{rpm_value:>{rpm_width}d}"
        
        return [
            f"Fan:{fan_text}",
            f"RPM: {rpm_text}",
            "",
    ]

    def _motor_names(self) -> list[str]:
        return ["Pulley", "Spool"]

    def _current_motor_tag(self) -> str:
        return "Pul" if self.state.pid_motor_index == 0 else "Spo"

    def _current_motor_name(self) -> str:
        return self._motor_names()[self.state.pid_motor_index]

    def _current_gains(self) -> PidGains:
        return self.state.pulley_gains if self.state.pid_motor_index == 0 else self.state.spool_gains

    def _gain_by_label(self, label: str) -> GainSetting:
        gains = self._current_gains()
        if label == "P":
            return gains.p
        if label == "I":
            return gains.i
        return gains.d

    def _pid_motor_line(self, motor_name: str, focused: bool, editing: bool, blink_on: bool) -> str:
        if editing:
            inner = motor_name if blink_on else " " * len(motor_name)
            return f"Motor:[{inner}]"
        if focused:
            return f"Motor:{self._focused_bracket(motor_name, blink_on)}"
        return f"Motor: {motor_name}"

    def _active_digit_for_label(self, label: str) -> int | None:
        if not self.state.menu_edit or self.state.edit_target != label:
            return None
        if self.state.gain_edit_step == 0:
            return None
        return self.state.gain_edit_step - 1

    def _adjust_gain_digit(self, gain: GainSetting, digit_index_from_left: int, step: int) -> None:
        digits = list(f"{gain.digits:05d}")
        current = int(digits[digit_index_from_left])
        digits[digit_index_from_left] = str((current + step) % 10)
        gain.digits = int("".join(digits))

    def _fan_speed_rpm(self) -> int:
        return int(round(self.state.fan_rpm))

    def _render_gain_value(
        self,
        gain: GainSetting,
        active_digit: int | None = None,
        blink_on: bool = True,
        dot_active: bool = False,
    ) -> str:
        digits = f"{gain.digits:05d}"
        idx = max(0, min(5, gain.dot_index))

        if idx == 5 and not dot_active:
            rendered = digits
        else:
            rendered = f"{digits[:idx]}.{digits[idx:]}"

        if active_digit is None and not dot_active:
            return rendered

        if active_digit is not None:
            visible = self._blink_digit(rendered, active_digit, blink_on)
        else:
            visible = self._blink_dot(rendered, blink_on)

        return f"[{visible}]"

    def _pid_gain_line(
        self,
        label: str,
        gain: GainSetting,
        active_digit: int | None,
        dot_active: bool,
        blink_on: bool,
        focused: bool = False,
    ) -> str:
        return self._pid_gain_segment(
            label,
            gain,
            active_digit,
            dot_active,
            blink_on,
            focused,
        )

    def _pid_gain_segment(
        self,
        label: str,
        gain: GainSetting,
        active_digit: int | None,
        dot_active: bool,
        blink_on: bool,
        focused: bool = False,
    ) -> str:
        rendered = self._render_gain_value(
            gain,
            active_digit,
            blink_on=blink_on,
            dot_active=dot_active,
        )
        if active_digit is not None or dot_active:
            return f"{label}:{rendered}"
        if focused:
            return f"{label}:{self._focused_bracket(rendered, blink_on)}"
        return f"{label}: {rendered}"

    def _pid_motor_prefix(self, motor_tag: str, focused: bool, editing: bool) -> str:
        blink_on = self._blink_on()
        if editing:
            return f"M:[{motor_tag}]"
        if focused:
            return f"M:{self._focused_bracket(motor_tag, blink_on)}"
        return f"M:{motor_tag}"

    def _focused_bracket(self, text: str, blink_on: bool) -> str:
        if blink_on:
            return f"[{text}]"
        return f" {text} "

    def _blink_digit(self, text: str, active_digit: int, blink_on: bool) -> str:
        if blink_on:
            return text

        digit_positions = [index for index, char in enumerate(text) if char.isdigit()]
        if active_digit < 0 or active_digit >= len(digit_positions):
            return text

        target_index = digit_positions[active_digit]
        return f"{text[:target_index]} {text[target_index + 1:]}"

    def _blink_dot(self, text: str, blink_on: bool) -> str:
        if blink_on:
            return text

        dot_index = text.find(".")
        if dot_index < 0:
            return text

        return f"{text[:dot_index]} {text[dot_index + 1:]}"
