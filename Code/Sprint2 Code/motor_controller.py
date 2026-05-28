"""Motor control wrapper that follows the Freenove Motor API."""
from __future__ import annotations
import time
from typing import Tuple
import config


class MotorController:
    """Thin wrapper around Freenove's Motor class - front wheels only."""

    def __init__(self) -> None:
        try:
            from Motor import Motor  # type: ignore
            self._motor = Motor()
            self._set_model = self._motor.setMotorModel
        except Exception:
            try:
                from motor import Ordinary_Car  # type: ignore
                self._motor = Ordinary_Car()
                self._set_model = self._motor.set_motor_model
            except Exception as exc:
                raise RuntimeError("Freenove Motor module not found.") from exc
        self._last_command: Tuple[int, int, int, int] = (0, 0, 0, 0)

    def set_raw(self, m1: int, m2: int, m3: int, m4: int) -> None:
        """Send motor values but force rear wheels to zero.
        Applies LEFT_TRIM to the left front wheel to correct veering.
        """
        if m1 > 0:
            m1 = int(m1 * config.LEFT_TRIM)
        elif m1 < 0:
            m1 = int(m1 * config.LEFT_TRIM)
        self._set_model(m1, 0, m3, 0)
        self._last_command = (m1, 0, m3, 0)

    def stop(self) -> None:
        self.set_raw(0, 0, 0, 0)

    def forward(self, speed: int) -> None:
        self.set_raw(speed, 0, speed, 0)

    def backward(self, speed: int) -> None:
        self.set_raw(-speed, 0, -speed, 0)

    def turn_left(self, speed: int) -> None:
        self.set_raw(-speed, 0, speed, 0)

    def turn_right(self, speed: int) -> None:
        self.set_raw(speed, 0, -speed, 0)

    def brake_with_delay(self, delay_s: float = 0.05) -> None:
        self.stop()
        time.sleep(delay_s)