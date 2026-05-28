"""
Motor adapter for the Freenove 4WD Smart Car.

Translates PathFollower output:
    (left_speed 0–100, left_fwd, right_speed 0–100, right_fwd)
Into Freenove Motor API calls:
    set_motor_model(left_front, left_rear, right_front, right_rear)
    Values: -4095 to +4095.  Positive = forward.

Set config.USE_ALL_WHEELS = True to drive all 4 motors.
Set False if rear motors are physically disconnected.
"""

import logging
import config

log = logging.getLogger(__name__)


class MotorAdapter:
    """Freenove motor interface for the path follower."""

    def __init__(self):
        self._car = None
        try:
            from motor import Ordinary_Car
            self._car = Ordinary_Car()
            log.info("Motor driver initialised (Ordinary_Car)")
        except Exception as exc:
            log.error(f"Motor init failed: {exc}")
            log.warning("Running in DRY-RUN mode — no motors will move")

    def set_motors(self, left_speed: float, left_fwd: bool,
                   right_speed: float, right_fwd: bool) -> None:
        """
        Send motor commands.

        Parameters
        ----------
        left_speed, right_speed : float  0–100 scale
        left_fwd, right_fwd : bool       True = forward
        """
        max_pwm = config.MAX_MOTOR_SPEED  # 4095

        # Scale 0–100 → 0–max_pwm
        lv = int(left_speed / 100.0 * max_pwm)
        rv = int(right_speed / 100.0 * max_pwm)

        # Apply direction
        if not left_fwd:
            lv = -lv
        if not right_fwd:
            rv = -rv

        # Apply trim to correct veering
        lv = int(lv * config.LEFT_TRIM)

        if config.USE_ALL_WHEELS:
            # All four wheels — better traction and turning
            self._send(lv, lv, rv, rv)
        else:
            # Front wheels only (rears = 0)
            self._send(lv, 0, rv, 0)

    def forward(self, speed: float = 50.0) -> None:
        self.set_motors(speed, True, speed, True)

    def stop(self) -> None:
        self._send(0, 0, 0, 0)

    def _send(self, m1: int, m2: int, m3: int, m4: int) -> None:
        if self._car is not None:
            self._car.set_motor_model(m1, m2, m3, m4)
        else:
            if any(v != 0 for v in (m1, m2, m3, m4)):
                log.debug(f"[DRY-RUN] motors: LF={m1} LR={m2} RF={m3} RR={m4}")
