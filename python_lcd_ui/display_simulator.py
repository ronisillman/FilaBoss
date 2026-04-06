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
        self._cell_size = 28
        self._cell_gap = 3
        self._cell_outline = "#447e4e"
        self._cell_fill = "#18341f"
        self._cell_text = "#a8ffb5"

        self._root = tk.Tk()
        self._root.title(f"LCD Simulator ({cols}x{rows})")
        self._root.configure(bg="#0b140e")
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        instruction = tk.Label(
            self._root,
            text="Controls: Left=up, Right=down, Up=down, Down=up, Space/Enter=select, D=Target Dia, S=Target Spd, Esc=Quit",
            fg="#b8ffc9",
            bg="#0b140e",
            font=("Consolas", 10),
            anchor="w",
            justify="left",
        )
        instruction.pack(fill="x", padx=10, pady=(8, 2))

        frame = tk.Frame(self._root, bg="#1d4f2b", bd=4, relief="ridge")
        frame.pack(padx=10, pady=(2, 10), fill="both", expand=True)

        canvas_width = self.cols * self._cell_size + (self.cols + 1) * self._cell_gap
        canvas_height = self.rows * self._cell_size + (self.rows + 1) * self._cell_gap
        self._canvas = tk.Canvas(
            frame,
            width=canvas_width,
            height=canvas_height,
            bg="#1d4f2b",
            highlightthickness=0,
        )
        self._canvas.pack(padx=8, pady=8)

        self._cell_items: list[list[tuple[int, int]]] = []
        for row in range(self.rows):
            row_items: list[tuple[int, int]] = []
            for col in range(self.cols):
                x1 = self._cell_gap + col * (self._cell_size + self._cell_gap)
                y1 = self._cell_gap + row * (self._cell_size + self._cell_gap)
                x2 = x1 + self._cell_size
                y2 = y1 + self._cell_size
                rect_id = self._canvas.create_rectangle(
                    x1,
                    y1,
                    x2,
                    y2,
                    outline=self._cell_outline,
                    fill=self._cell_fill,
                    width=2,
                )
                text_id = self._canvas.create_text(
                    (x1 + x2) / 2,
                    (y1 + y2) / 2,
                    text=" ",
                    fill=self._cell_text,
                    font=("Consolas", 15, "bold"),
                )
                row_items.append((rect_id, text_id))
            self._cell_items.append(row_items)

        self._root.bind("<Up>", lambda _e: self._events.append(InputEvent("down")))
        self._root.bind("<Left>", lambda _e: self._events.append(InputEvent("up")))
        self._root.bind("<Down>", lambda _e: self._events.append(InputEvent("up")))
        self._root.bind("<Right>", lambda _e: self._events.append(InputEvent("down")))
        self._root.bind("<space>", lambda _e: self._events.append(InputEvent("select")))
        self._root.bind("<Return>", lambda _e: self._events.append(InputEvent("select")))
        self._root.bind("<d>", lambda _e: self._events.append(InputEvent("target_dia")))
        self._root.bind("<D>", lambda _e: self._events.append(InputEvent("target_dia")))
        self._root.bind("<s>", lambda _e: self._events.append(InputEvent("target_spd")))
        self._root.bind("<S>", lambda _e: self._events.append(InputEvent("target_spd")))
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
        for row, line in enumerate(self._buffer):
            self._render_row(row, line)

    def write_line(self, row: int, text: str) -> None:
        if row < 0 or row >= self.rows:
            return
        rendered = (text[: self.cols]).ljust(self.cols)
        if rendered == self._buffer[row]:
            return
        self._buffer[row] = rendered
        self._render_row(row, rendered)

    def _render_row(self, row: int, text: str) -> None:
        cell_row = self._cell_items[row]
        for col, char in enumerate(text):
            _, text_id = cell_row[col]
            self._canvas.itemconfigure(text_id, text=char)

    def close(self) -> None:
        self._open = False
        try:
            self._root.destroy()
        except tk.TclError:
            pass
