#!/usr/bin/env python3
"""
Test ultrasonic sensor WHILE motors are running.
Robot drives forward slowly while printing distance readings.
Hold an obstacle in front to see if it detects it.
Press Ctrl+C to stop.
"""

import time
from motor_adapter import MotorAdapter
from ultrasonic import UltrasonicSensor

motor = MotorAdapter()
sensor = UltrasonicSensor()

if not sensor.available:
    print("Ultrasonic sensor not available!")
    exit(1)

print("Driving forward at low speed while reading ultrasonic...")
print("Hold an obstacle in front of the robot.")
print("Press Ctrl+C to stop.\n")

try:
    motor.set_motors(30, True, 30, True)  # slow forward

    while True:
        dist = sensor.measure_cm()
        bar = "#" * min(int(dist / 2), 50)
        status = " *** OBSTACLE! ***" if dist < 35 else ""
        print(f"  {dist:6.1f} cm  {bar}{status}")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopped")
finally:
    motor.stop()
