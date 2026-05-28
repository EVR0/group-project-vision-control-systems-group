#!/usr/bin/env python3
"""
Simple motor test — forward, backward, turn left, turn right.
Each movement runs for 1 second at low speed.
Press Ctrl+C at any time to stop.
"""

import time
from motor_adapter import MotorAdapter

motor = MotorAdapter()
speed = 30.0  # Low speed for safety

try:
    print("Forward...")
    motor.set_motors(speed, True, speed, True)
    time.sleep(1)
    motor.stop()
    time.sleep(1)

    print("Backward...")
    motor.set_motors(speed, False, speed, False)
    time.sleep(1)
    motor.stop()
    time.sleep(1)

    print("Turn left (spin in place)...")
    motor.set_motors(speed, False, speed, True)
    time.sleep(1)
    motor.stop()
    time.sleep(1)

    print("Turn right (spin in place)...")
    motor.set_motors(speed, True, speed, False)
    time.sleep(1)
    motor.stop()
    time.sleep(1)

    print("Done! All motors working.")

except KeyboardInterrupt:
    print("\nStopped")
finally:
    motor.stop()
