"""
Ultrasonic distance sensor for Freenove 4WD Smart Car.

Uses HC-SR04 sensor connected to GPIO:
  - Trigger: GPIO 27
  - Echo:    GPIO 22

These are the standard Freenove 4WD pins. Adjust if your wiring differs.
"""

import time
import logging
import config

log = logging.getLogger(__name__)

try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False
    log.warning("RPi.GPIO not available — ultrasonic sensor disabled")

# Default Freenove 4WD ultrasonic pins
TRIG_PIN = 27
ECHO_PIN = 22

# Measurement limits
MAX_DISTANCE_CM = 300.0
TIMEOUT_S = 0.04  # ~400cm round-trip at speed of sound


class UltrasonicSensor:
    """HC-SR04 ultrasonic distance sensor."""

    def __init__(self, trig_pin=None, echo_pin=None):
        if trig_pin is None:
            trig_pin = config.ULTRASONIC_TRIG_PIN if hasattr(config, "ULTRASONIC_TRIG_PIN") else TRIG_PIN
        if echo_pin is None:
            echo_pin = config.ULTRASONIC_ECHO_PIN if hasattr(config, "ULTRASONIC_ECHO_PIN") else ECHO_PIN
        self.trig = trig_pin
        self.echo = echo_pin
        self._available = False

        if not _GPIO_AVAILABLE:
            return

        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trig, GPIO.OUT)
            GPIO.setup(self.echo, GPIO.IN)
            GPIO.output(self.trig, False)
            time.sleep(0.1)
            self._available = True
            log.info(f"Ultrasonic sensor ready (trig=GPIO{trig_pin}, echo=GPIO{echo_pin})")
        except Exception as e:
            log.warning(f"Ultrasonic sensor init failed: {e}")

    def measure_cm(self) -> float:
        """
        Take one distance measurement.
        Returns distance in centimetres, or MAX_DISTANCE_CM on timeout/error.
        """
        if not self._available:
            return MAX_DISTANCE_CM

        try:
            # Send 10us trigger pulse
            GPIO.output(self.trig, True)
            time.sleep(0.00001)
            GPIO.output(self.trig, False)

            # Wait for echo to go HIGH (start of return pulse)
            start = time.monotonic()
            deadline = start + TIMEOUT_S
            while GPIO.input(self.echo) == 0:
                start = time.monotonic()
                if start > deadline:
                    return MAX_DISTANCE_CM

            # Wait for echo to go LOW (end of return pulse)
            end = start
            while GPIO.input(self.echo) == 1:
                end = time.monotonic()
                if end > deadline:
                    return MAX_DISTANCE_CM

            # Distance = time * speed_of_sound / 2
            elapsed = end - start
            distance_cm = (elapsed * 34300.0) / 2.0

            return min(distance_cm, MAX_DISTANCE_CM)

        except Exception as e:
            log.debug(f"Ultrasonic measurement error: {e}")
            return MAX_DISTANCE_CM

    def measure_avg_cm(self, samples=3, delay=0.02) -> float:
        """Take multiple measurements and return the median."""
        readings = []
        for _ in range(samples):
            readings.append(self.measure_cm())
            time.sleep(delay)
        readings.sort()
        return readings[len(readings) // 2]  # median

    @property
    def available(self) -> bool:
        return self._available

    def cleanup(self):
        """Release GPIO pins."""
        if self._available:
            try:
                GPIO.cleanup((self.trig, self.echo))
            except Exception:
                pass
