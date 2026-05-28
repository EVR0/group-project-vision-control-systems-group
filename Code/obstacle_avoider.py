"""
Reactive obstacle avoidance — sharp turns, very short choppy movements.
Checks sensor constantly between every micro-movement.
"""

import time
import logging
from motor_adapter import MotorAdapter
import config

log = logging.getLogger(__name__)

_PAUSE_S = 0.15
_TURN_BURST_S = 0.15       # Very short turn bursts
_TURN_SPEED = 65.0          # Sharp fast turns
_DRIVE_SPEED = 50.0
_DRIVE_BURST_S = 0.10       # Very short drive bursts
_MAX_TURN_ATTEMPTS = 10
_DRIVE_BURSTS = 6

EMERGENCY_DIST_CM = 20.0
BACKUP_SPEED = 45.0
BACKUP_TIME_S = 0.35


class ObstacleAvoider:

    def __init__(self, sensor, motor: MotorAdapter):
        self.sensor = sensor
        self.motor = motor
        self._turn_direction = 1
        self._avoiding = False

    @property
    def is_avoiding(self) -> bool:
        return self._avoiding

    def _read_distance(self) -> float:
        readings = []
        for _ in range(3):
            readings.append(self.sensor.measure_cm())
            time.sleep(0.005)
        readings.sort()
        return readings[1]

    def check_obstacle(self) -> bool:
        if not self.sensor.available:
            return False
        readings = []
        for _ in range(5):
            readings.append(self.sensor.measure_cm())
            time.sleep(0.005)
        readings.sort()
        dist = readings[1]
        if dist < config.OBSTACLE_THRESHOLD_CM:
            log.info(f"OBSTACLE at {dist:.1f} cm!")
            return True
        return False

    def _emergency_backup(self):
        log.warning("EMERGENCY — too close! Backing up!")
        self.motor.stop()
        time.sleep(0.1)
        self.motor.set_motors(BACKUP_SPEED, False, BACKUP_SPEED, False)
        time.sleep(BACKUP_TIME_S)
        self.motor.stop()
        time.sleep(_PAUSE_S)

    def execute_avoidance(self) -> None:
        self._avoiding = True
        direction = self._turn_direction
        dir_name = "RIGHT" if direction > 0 else "LEFT"

        log.info(f"=== AVOIDANCE: turning {dir_name} ===")

        try:
            # Emergency backup if too close
            dist = self._read_distance()
            if dist < EMERGENCY_DIST_CM:
                self._emergency_backup()

            self.motor.stop()
            time.sleep(_PAUSE_S)

            # Turn in sharp short bursts until clear
            cleared = False
            for i in range(1, _MAX_TURN_ATTEMPTS + 1):
                # Sharp turn burst
                if direction > 0:
                    self.motor.set_motors(_TURN_SPEED, True, _TURN_SPEED, False)
                else:
                    self.motor.set_motors(_TURN_SPEED, False, _TURN_SPEED, True)
                time.sleep(_TURN_BURST_S)
                self.motor.stop()
                time.sleep(_PAUSE_S)

                # Check
                dist = self._read_distance()
                log.info(f"  Turn {i}: {dist:.1f} cm")

                # Emergency
                if dist < EMERGENCY_DIST_CM:
                    self._emergency_backup()
                    continue

                if dist >= config.OBSTACLE_THRESHOLD_CM:
                    log.info(f"  Path clear!")
                    cleared = True
                    break

            if not cleared:
                log.warning("  Max turns reached — forcing past")

            # Drive past in very short choppy bursts
            log.info("  Driving past...")
            for i in range(_DRIVE_BURSTS):
                dist = self._read_distance()

                if dist < EMERGENCY_DIST_CM:
                    self._emergency_backup()
                    # Turn more
                    if direction > 0:
                        self.motor.set_motors(_TURN_SPEED, True, _TURN_SPEED, False)
                    else:
                        self.motor.set_motors(_TURN_SPEED, False, _TURN_SPEED, True)
                    time.sleep(_TURN_BURST_S)
                    self.motor.stop()
                    time.sleep(_PAUSE_S)
                    continue

                if dist < config.OBSTACLE_THRESHOLD_CM:
                    log.info(f"  Blocked ({dist:.1f} cm) - turning more")
                    if direction > 0:
                        self.motor.set_motors(_TURN_SPEED, True, _TURN_SPEED, False)
                    else:
                        self.motor.set_motors(_TURN_SPEED, False, _TURN_SPEED, True)
                    time.sleep(_TURN_BURST_S)
                    self.motor.stop()
                    time.sleep(_PAUSE_S)
                    continue

                # Short drive burst
                self.motor.set_motors(_DRIVE_SPEED, True, _DRIVE_SPEED, True)
                time.sleep(_DRIVE_BURST_S)
                self.motor.stop()
                time.sleep(0.08)

            self.motor.stop()
            self._turn_direction *= -1
            log.info("=== AVOIDANCE COMPLETE ===")

        except Exception as e:
            log.error(f"Avoidance error: {e}")
            self.motor.stop()
        finally:
            self._avoiding = False
