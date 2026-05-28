#!/usr/bin/env python3
"""
Test ultrasonic sensor — prints distance readings.
Hold objects at different distances to verify readings.
Press Ctrl+C to stop.
"""

import time
from ultrasonic import UltrasonicSensor

sensor = UltrasonicSensor()

if not sensor.available:
    print("Ultrasonic sensor not available!")
    print("Check wiring: Trig=GPIO27, Echo=GPIO22")
    exit(1)

print("Ultrasonic sensor ready. Showing distance readings...")
print("Hold objects in front of sensor to test.")
print("Press Ctrl+C to stop.\n")

try:
    while True:
        dist = sensor.measure_cm()
        avg = sensor.measure_avg_cm(samples=3)
        bar = "#" * min(int(avg / 2), 50)
        print(f"  {avg:6.1f} cm  {bar}")
        time.sleep(0.2)
except KeyboardInterrupt:
    print("\nStopped")
finally:
    sensor.cleanup()
