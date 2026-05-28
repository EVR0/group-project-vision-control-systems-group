#!/usr/bin/env python3
"""
Test AprilTag detection and localization.

Place the robot on a known tag, run this script, and verify:
  - Tag IDs are detected correctly
  - Reported position matches the tag's known position
  - Moving the robot right increases X, moving it down increases Y
  - Rotating clockwise increases heading

Press Ctrl+C to stop.
"""

from picamera2 import Picamera2
from libcamera import Transform, controls
import cv2
import time

import config
from detector import AprilTagDetector
from localization import Localizer
from tag_layout import TAG_LAYOUT

# --- Camera setup (matches main.py) ---
cam = Picamera2()
cfg = cam.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    raw={"size": (2304, 1296)},
    transform=Transform(hflip=config.CAMERA_HFLIP, vflip=config.CAMERA_VFLIP),
)
cam.configure(cfg)
cam.start()

ctrl = {"ScalerCrop": (0, 0, 4608, 2592)}
if config.CAMERA_FOCUS_MODE == "manual":
    ctrl["AfMode"] = controls.AfModeEnum.Manual
    ctrl["LensPosition"] = config.CAMERA_LENS_POSITION
else:
    ctrl["AfMode"] = controls.AfModeEnum.Continuous
cam.set_controls(ctrl)

time.sleep(2)
print("Camera ready\n")

detector = AprilTagDetector()
localizer = Localizer()

print("Looking for tags... (Ctrl+C to stop)")
print("=" * 65)
print()

try:
    while True:
        frame = cam.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        detections = detector.detect(gray)

        if detections:
            # Show raw detections
            print(f"--- {len(detections)} tag(s) detected ---")
            for d in detections:
                dist = float((d.tvec[0]**2 + d.tvec[1]**2 + d.tvec[2]**2) ** 0.5)
                known = TAG_LAYOUT.get(int(d.tag_id))
                known_str = (f"  known=({known[0]:.4f}, {known[1]:.4f})"
                             if known else "  (UNKNOWN TAG)")
                print(f"  Tag {d.tag_id:>3d}  dist={dist:.3f}m"
                      f"  margin={d.decision_margin:.1f}{known_str}")

            # Show localized pose
            pose = localizer.compute_pose(detections)
            if pose:
                print(f"  >>> POSE: x={pose['board_x']:.4f}  y={pose['board_y']:.4f}"
                      f"  yaw={pose['board_yaw']:.1f}°"
                      f"  nearest=tag{pose['closest_tag_id']}"
                      f"  conf={pose['confidence']:.2f}")
            print()
        else:
            print("  (no tags)")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nStopped")
finally:
    cam.stop()
