#!/usr/bin/env python3
"""
Diagnostic: do ONE short turn burst, measure before/after heading.
This tells us exactly how much the robot turns per burst,
and whether the direction is correct.
"""

import time
import math
import cv2
from picamera2 import Picamera2
from libcamera import Transform, controls

import config
from detector import AprilTagDetector
from localization import Localizer
from motor_adapter import MotorAdapter


def get_heading(cam, detector, localizer, attempts=20):
    """Try to get a heading, retrying up to `attempts` times."""
    for _ in range(attempts):
        frame = cam.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        dets = detector.detect(gray)
        if dets:
            pose = localizer.compute_pose(dets)
            if pose:
                return pose["board_yaw"], pose["board_x"], pose["board_y"]
        time.sleep(0.1)
    return None, None, None


def main():
    # Init
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

    detector = AprilTagDetector()
    localizer = Localizer()
    motor = MotorAdapter()
    
    turn_speed = 30.0
    burst_time = 0.20  # seconds

    try:
        # 1. Get starting heading
        print("Getting starting heading...")
        yaw0, x0, y0 = get_heading(cam, detector, localizer)
        if yaw0 is None:
            print("ERROR: Can't see any tags! Point camera at floor tags.")
            return
        print(f"  Start: yaw={yaw0:.1f}  pos=({x0:.4f}, {y0:.4f})")

        # 2. Do one RIGHT turn burst
        print(f"\nTurning RIGHT for {burst_time}s at speed {turn_speed}...")
        motor.set_motors(turn_speed, True, turn_speed, False)  # left fwd, right back
        time.sleep(burst_time)
        motor.stop()

        # 3. Wait for camera to re-acquire
        print("Waiting for camera...")
        time.sleep(1.0)

        # 4. Get new heading
        yaw1, x1, y1 = get_heading(cam, detector, localizer)
        if yaw1 is None:
            print("ERROR: Can't see tags after turn! Robot may have turned off the board.")
            return
        print(f"  After: yaw={yaw1:.1f}  pos=({x1:.4f}, {y1:.4f})")

        delta = yaw1 - yaw0
        if delta > 180: delta -= 360
        if delta < -180: delta += 360
        print(f"\n  Yaw change: {delta:+.1f} degrees")
        if delta > 0:
            print("  Direction: CLOCKWISE (correct for right turn)")
        else:
            print("  Direction: COUNTER-CLOCKWISE (WRONG for right turn!)")

        # 5. Now do one LEFT turn burst
        print(f"\nTurning LEFT for {burst_time}s at speed {turn_speed}...")
        motor.set_motors(turn_speed, False, turn_speed, True)  # left back, right fwd
        time.sleep(burst_time)
        motor.stop()

        print("Waiting for camera...")
        time.sleep(1.0)

        yaw2, x2, y2 = get_heading(cam, detector, localizer)
        if yaw2 is None:
            print("ERROR: Can't see tags after turn!")
            return
        print(f"  After: yaw={yaw2:.1f}  pos=({x2:.4f}, {y2:.4f})")

        delta2 = yaw2 - yaw1
        if delta2 > 180: delta2 -= 360
        if delta2 < -180: delta2 += 360
        print(f"\n  Yaw change: {delta2:+.1f} degrees")
        if delta2 < 0:
            print("  Direction: COUNTER-CLOCKWISE (correct for left turn)")
        else:
            print("  Direction: CLOCKWISE (WRONG for left turn!)")

        # 6. Now do a short FORWARD burst
        print(f"\nDriving FORWARD for {burst_time}s at speed {turn_speed}...")
        motor.set_motors(turn_speed, True, turn_speed, True)
        time.sleep(burst_time)
        motor.stop()

        print("Waiting for camera...")
        time.sleep(1.0)

        yaw3, x3, y3 = get_heading(cam, detector, localizer)
        if yaw3 is None:
            print("ERROR: Can't see tags after driving!")
            return
        print(f"  After: yaw={yaw3:.1f}  pos=({x3:.4f}, {y3:.4f})")

        dx = x3 - x2
        dy = y3 - y2
        dist = math.sqrt(dx*dx + dy*dy)
        print(f"\n  Moved: dx={dx:+.4f}  dy={dy:+.4f}  dist={dist:.4f}m")

        print("\n=== SUMMARY ===")
        print(f"Right turn {burst_time}s: {abs(delta):.1f} degrees/burst")
        print(f"Left turn  {burst_time}s: {abs(delta2):.1f} degrees/burst")
        print(f"Forward    {burst_time}s: {dist*100:.1f} cm")

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        motor.stop()
        cam.stop()


if __name__ == "__main__":
    main()
