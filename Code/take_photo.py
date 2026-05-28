#!/usr/bin/env python3
"""Capture one frame from the camera and save it as test_frame.jpg."""

from picamera2 import Picamera2
import cv2
import time

cam = Picamera2()
cfg = cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
cam.configure(cfg)
cam.start()
time.sleep(2)  # let auto-exposure settle

frame = cam.capture_array()
cv2.imwrite("test_frame.jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
cam.stop()

print("Saved test_frame.jpg")
print("Download it to your PC to see what the camera sees:")
print("  In MobaXterm: use the file browser on the left sidebar")
print("  Or run: scp pi@car-9:~/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server/test_frame.jpg .")
