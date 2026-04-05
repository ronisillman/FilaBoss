from __future__ import annotations

# pyright: reportMissingImports=false

from display_base import DisplayBackend


class HardwareLcdDisplay(DisplayBackend):
    """HD44780 character LCD over PCF8574(T) using RPLCD."""

    def __init__(
        self,
        cols: int = 20,
        rows: int = 4,
        i2c_expander: str = "PCF8574",
        i2c_address: int = 0x27,
        i2c_port: int = 1,
    ) -> None:
        self.cols = cols
        self.rows = rows
        try:
            from RPLCD.i2c import CharLCD
        except ImportError as exc:
            raise RuntimeError(
                "RPLCD is not installed. Run: pip install RPLCD smbus2"
            ) from exc

        self._lcd = CharLCD(
            i2c_expander=i2c_expander,
            address=i2c_address,
            port=i2c_port,
            cols=cols,
            rows=rows,
            charmap="A02",
            auto_linebreaks=False,
        )
        self._buffer = [" " * self.cols for _ in range(self.rows)]
        self.clear()

    def clear(self) -> None:
        self._lcd.clear()
        self._buffer = [" " * self.cols for _ in range(self.rows)]

    def write_line(self, row: int, text: str) -> None:
        if row < 0 or row >= self.rows:
            return
        rendered = (text[: self.cols]).ljust(self.cols)
        if rendered == self._buffer[row]:
            return

        self._lcd.cursor_pos = (row, 0)
        self._lcd.write_string(rendered)
        self._buffer[row] = rendered

    def close(self) -> None:
        try:
            self._lcd.clear()
        finally:
            self._lcd.close(clear=False)
