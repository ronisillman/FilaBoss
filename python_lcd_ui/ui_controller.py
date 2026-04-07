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
    p: GainSetting = field(default_factory=lambda: GainSetting(5000, 2))
    i: GainSetting = field(default_factory=lambda: GainSetting(1500, 2))
    d: GainSetting = field(default_factory=lambda: GainSetting(5, 5))


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
    target_mode: str = "Dia"
    target_diameter_hundredths: int = 175
    target_speed_tenths: int = 120
    load_mode: bool = False
    pulley_speed_mps: float = 10.0
    diameter_travelled_mm: float = 0.0
    pulley_gains: PidGains = field(default_factory=PidGains)
    spool_gains: PidGains = field(
        default_factory=lambda: PidGains(
            p=GainSetting(7000, 2),
            i=GainSetting(2500, 2),
            d=GainSetting(5, 5),
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

        if event.kind == "load_toggle":
            self.state.load_mode = not self.state.load_mode
            self.state.menu_edit = False
            self.state.edit_target = ""
            self.state.gain_edit_step = 0
            return True

        if event.kind == "target_dia":
            self.state.target_mode = "Dia"
            return True

        if event.kind == "target_spd":
            self.state.target_mode = "Spd"
            return True

        if event.kind == "select":
            self._handle_select()
            return True

        if event.kind in {"up", "down"}:
            self._handle_turn(event.kind)
            return True

        return True

    def _handle_select(self) -> None:
        if self.state.load_mode:
            return

        if self.state.menu_edit:
            if self.state.edit_target in {"P", "I", "D"}:
                if self.state.gain_edit_step < 5:
                    self.state.gain_edit_step += 1
                    return
            elif self.state.edit_target == "MAIN_TRGT":
                if self.state.gain_edit_step < 3:
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

        if focus_item in {"MOTOR", "P", "I", "D", "FAN_SPEED", "MAIN_TRGT"}:
            self.state.menu_edit = True
            self.state.edit_target = focus_item
            self.state.gain_edit_step = 1 if focus_item == "MAIN_TRGT" else 0

    def _handle_turn(self, direction: str) -> None:
        step = -1 if direction == "up" else 1

        if self.state.load_mode:
            self.state.target_speed_tenths = max(0, min(999, self.state.target_speed_tenths + step))
            return

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

        if target == "MAIN_TRGT":
            digit_index = max(0, min(2, self.state.gain_edit_step - 1))
            self._adjust_main_target_digit(digit_index, step)
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
        return ["MAIN", "PID", "FAN", "MAIN_TRGT"]

    def _current_focus_item(self) -> str:
        items = self._menu_items()
        if not items:
            return ""
        idx = max(0, min(self.state.focus_index, len(items) - 1))
        return items[idx]

    def tick(self, simulate_feedback: bool = True) -> None:
        if not simulate_feedback:
            return

        elapsed = time.monotonic() - self._start
        diameter_wave = 1.75 + math.sin(elapsed * 0.35) * 0.02
        fan_trim = (self.state.fan_speed_pct - 50) * 0.02
        self.state.filament_diameter_mm = diameter_wave

        actual_wave = math.sin(elapsed * 1.4) * 0.2
        target_speed_mps = self.state.target_speed_tenths / 10.0
        self.state.pulley_speed_mps = max(0.0, target_speed_mps * 0.98 + actual_wave)

        target_fan_rpm = self.state.fan_speed_pct * 20.0
        raw_offset = math.sin(elapsed * 0.55 + 0.6) * 20.0
        stepped_offset = round(raw_offset / 20.0) * 20.0
        stepped_offset = max(-20.0, min(20.0, stepped_offset))
        self.state.fan_rpm = max(0.0, min(2000.0, target_fan_rpm + stepped_offset))

    def _blink_on(self) -> bool:
        elapsed = time.monotonic() - self._start
        return int(elapsed * 3) % 2 == 0

    def render_lines(self) -> list[str]:
        if self.state.load_mode:
            return self._render_load_mode()

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
        chars = [" "] * self.cols

        self._write_tab(chars, 0, "MAIN", focused=(focus_item == "MAIN"), blink_on=blink_on)
        self._write_tab(chars, 6, "PID", focused=(focus_item == "PID"), blink_on=blink_on)
        self._write_tab(chars, 11, "FAN", focused=(focus_item == "FAN"), blink_on=blink_on)

        return "".join(chars)

    def _write_tab(self, chars: list[str], start_index: int, label: str, focused: bool, blink_on: bool) -> None:
        width = len(label) + 2

        if not focused:
            rendered = f" {label} "
        elif blink_on:
            rendered = f"[{label}]"
        else:
            rendered = f" {label} "

        for offset, char in enumerate(rendered[:width]):
            index = start_index + offset
            if 0 <= index < self.cols:
                chars[index] = char

    def _render_main_menu(self) -> list[str]:
        blink_on = self._blink_on()
        focus_item = self._current_focus_item() if not self.state.menu_edit else ""
        target_focused = focus_item == "MAIN_TRGT"
        target_editing = self.state.menu_edit and self.state.edit_target == "MAIN_TRGT"
        active_digit = self.state.gain_edit_step - 1 if target_editing else None
        target_value = self._render_main_target_value(
            active_digit=active_digit,
            blink_on=blink_on,
            focused=target_focused,
            editing=target_editing,
        )

        return [
            f"Dia: {self.state.filament_diameter_mm:4.2f} mm",
            f"Spd: {self.state.pulley_speed_mps:4.1f} mm/s",
            f"M: {self.state.target_mode}  Trgt:{target_value}",
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

    def _render_load_mode(self) -> list[str]:
        target_value = f"{self.state.target_speed_tenths / 10.0:4.1f} mm/s"
        actual_value = f"{self.state.pulley_speed_mps:4.1f} mm/s"

        return [
            "Load Mode Active",
            "Adjust Pulley Speed:",
            f"Target: {target_value}",
            f"Actual: {actual_value}",
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

    def _main_target_digits(self) -> str:
        if self.state.target_mode == "Dia":
            value = max(0, min(999, self.state.target_diameter_hundredths))
        else:
            value = max(0, min(999, self.state.target_speed_tenths))
        return f"{value:03d}"

    def _set_main_target_digits(self, digits: str) -> None:
        value = int(digits)
        if self.state.target_mode == "Dia":
            self.state.target_diameter_hundredths = value
        else:
            self.state.target_speed_tenths = value

    def _adjust_main_target_digit(self, digit_index_from_left: int, step: int) -> None:
        digits = list(self._main_target_digits())
        current = int(digits[digit_index_from_left])
        digits[digit_index_from_left] = str((current + step) % 10)
        self._set_main_target_digits("".join(digits))

    def _render_main_target_value(
        self,
        active_digit: int | None,
        blink_on: bool,
        focused: bool = False,
        editing: bool = False,
    ) -> str:
        digits = self._main_target_digits()
        if self.state.target_mode == "Dia":
            rendered = f"{digits[0]}.{digits[1:]}"
        else:
            rendered = f"{digits[:2]}.{digits[2]}"

        if active_digit is not None:
            visible = self._blink_digit(rendered, active_digit, blink_on)
            return f"[{visible}]"

        if editing or focused:
            if blink_on:
                return f"[{rendered}]"

        return f" {rendered}"

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
