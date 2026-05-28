"""
Motor driver for Freenove 4WD Smart Car — built on pca9685.py.

PCA9685 channel mapping (standard Freenove 4WD):
  Motor 0 (left front):  ch 0 = backward, ch 1 = forward
  Motor 1 (left rear):   ch 2 = backward, ch 3 = forward
  Motor 2 (right front): ch 4 = backward, ch 5 = forward
  Motor 3 (right rear):  ch 6 = backward, ch 7 = forward
"""

from pca9685 import PCA9685


class Ordinary_Car:
    """Freenove 4WD motor controller."""

    _MOTOR_CHANNELS = {
        0: (0, 1),   # left front
        1: (2, 3),   # left rear
        2: (6, 7),   # right front
        3: (4, 5),   # right rear
    }

    def __init__(self):
        self.pwm = PCA9685()
        self.pwm.set_pwm_freq(50)
        self.set_motor_model(0, 0, 0, 0)

    def _set_single_motor(self, motor_id: int, duty: int) -> None:
        bwd_ch, fwd_ch = self._MOTOR_CHANNELS[motor_id]
        duty = max(-4095, min(4095, int(duty)))

        if duty > 0:
            self.pwm.set_motor_pwm(bwd_ch, 0)
            self.pwm.set_motor_pwm(fwd_ch, duty)
        elif duty < 0:
            self.pwm.set_motor_pwm(fwd_ch, 0)
            self.pwm.set_motor_pwm(bwd_ch, abs(duty))
        else:
            self.pwm.set_motor_pwm(fwd_ch, 0)
            self.pwm.set_motor_pwm(bwd_ch, 0)

    def set_motor_model(self, left_front: int, left_rear: int,
                        right_front: int, right_rear: int) -> None:
        self._set_single_motor(0, left_front)
        self._set_single_motor(1, left_rear)
        self._set_single_motor(2, right_front)
        self._set_single_motor(3, right_rear)

    def close(self) -> None:
        self.set_motor_model(0, 0, 0, 0)
        self.pwm.close()
