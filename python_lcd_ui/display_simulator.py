from __future__ import annotations

import tkinter as tk
from collections import deque

from display_base import DisplayBackend
from input_devices import InputEvent


class SimulatedLcdDisplay(DisplayBackend):
    """Small desktop simulator that looks like a green character LCD."""

    def __init__(self, cols: int = 20, rows: int = 4) -> None:
        self.cols = cols
        self.rows = rows
        self._open = True
        self._events: deque[InputEvent] = deque()
        self._buffer = [" " * self.cols for _ in range(self.rows)]

        self._root = tk.Tk()
        self._root.title(f"LCD Simulator ({cols}x{rows})")
        self._root.configure(bg="#0b140e")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        instruction = tk.Label(
            self._root,
            text="Controls: Arrow Up/Left=UP, Arrow Down/Right=DOWN, Space=SELECT, Esc=Quit",
            fg="#b8ffc9",
            bg="#0b140e",
            font=("Consolas", 10),
            anchor="w",
            justify="left",
        )
        instruction.pack(fill="x", padx=10, pady=(8, 2))

        frame = tk.Frame(self._root, bg="#1d4f2b", bd=4, relief="ridge")
        frame.pack(padx=10, pady=(2, 10), fill="both", expand=True)

        self._line_labels: list[tk.Label] = []
        for _ in range(self.rows):
            lbl = tk.Label(
                frame,
                text=" " * self.cols,
                fg="#93f59f",
                bg="#1d4f2b",
                font=("Consolas", 16, "bold"),
                anchor="w",
                width=self.cols,
            )
            lbl.pack(padx=8, pady=2, anchor="w")
            self._line_labels.append(lbl)

        self._root.bind("<Up>", lambda _e: self._events.append(InputEvent("up")))
        self._root.bind("<Left>", lambda _e: self._events.append(InputEvent("up")))
        self._root.bind("<Down>", lambda _e: self._events.append(InputEvent("down")))
        self._root.bind("<Right>", lambda _e: self._events.append(InputEvent("down")))
        self._root.bind("<space>", lambda _e: self._events.append(InputEvent("select")))
        self._root.bind("<Return>", lambda _e: self._events.append(InputEvent("select")))
        self._root.bind("<Escape>", lambda _e: self._events.append(InputEvent("quit")))

        self._root.focus_force()
        self.clear()

    @property
    def is_open(self) -> bool:
        return self._open

    def _on_close(self) -> None:
        self._open = False

    def poll_events(self) -> list[InputEvent]:
        return_events: list[InputEvent] = []
        while self._events:
            return_events.append(self._events.popleft())
        return return_events

    def refresh_ui(self) -> None:
        if not self._open:
            return
        try:
            self._root.update_idletasks()
            self._root.update()
        except tk.TclError:
            self._open = False

    def clear(self) -> None:
        self._buffer = [" " * self.cols for _ in range(self.rows)]
        for i, line in enumerate(self._buffer):
            self._line_labels[i].configure(text=line)

    def write_line(self, row: int, text: str) -> None:
        if row < 0 or row >= self.rows:
            return
        rendered = (text[: self.cols]).ljust(self.cols)
        if rendered == self._buffer[row]:
            return
        self._buffer[row] = rendered
        self._line_labels[row].configure(text=rendered)

    def close(self) -> None:
        self._open = False
        try:
            self._root.destroy()
        except tk.TclError:
            pass
