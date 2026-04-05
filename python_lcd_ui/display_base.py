from __future__ import annotations

from abc import ABC, abstractmethod


class DisplayBackend(ABC):
    """Common interface for LCD display backends."""

    cols: int
    rows: int

    @abstractmethod
    def clear(self) -> None:
        pass

    @abstractmethod
    def write_line(self, row: int, text: str) -> None:
        pass

    @abstractmethod
    def close(self) -> None:
        pass
