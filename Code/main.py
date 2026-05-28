#!/usr/bin/env python3
"""
Freenove 4WD AprilTag Navigation — Main Loop
With ultrasonic obstacle avoidance and gap-crossing.
"""

import sys
import math
import time
import logging
import argparse

import config
from tag_layout import TAG_LAYOUT
from tag_map import TagMap
from detector import AprilTagDetector
from localization import Localizer
from pathfinder import plan_path
from controller import PathFollower
from motor_adapter import MotorAdapter
from ultrasonic import UltrasonicSensor
from obstacle_avoider import ObstacleAvoider

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-7s [%(name)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("nav")

TAG_LOSS_COAST_S = 0.3
TAG_LOSS_STOP_S = 1.5


def _wrap(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a


def _check_and_avoid(avoider, motor):
    """Check for obstacle and execute avoidance. Returns True if avoided."""
    if avoider.check_obstacle():
        motor.stop()
        avoider.execute_avoidance()
        return True
    return False


def init_camera():
    from picamera2 import Picamera2
    from libcamera import Transform, controls

    cam = Picamera2()
    cam_config = cam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        raw={"size": (2304, 1296)},
        transform=Transform(hflip=config.CAMERA_HFLIP, vflip=config.CAMERA_VFLIP),
    )
    cam.configure(cam_config)
    cam.start()

    ctrl = {"ScalerCrop": (0, 0, 4608, 2592)}
    if config.CAMERA_FOCUS_MODE == "manual":
        ctrl["AfMode"] = controls.AfModeEnum.Manual
        ctrl["LensPosition"] = config.CAMERA_LENS_POSITION
    else:
        ctrl["AfMode"] = controls.AfModeEnum.Continuous
    cam.set_controls(ctrl)

    time.sleep(2)
    log.info("Camera ready (640x480)")
    return cam


def init_servo():
    try:
        from Servo import Servo
        servo = Servo()
        setter = servo.setServoPwm
    except Exception:
        try:
            from servo import Servo
            servo = Servo()
            setter = lambda ch, ang: servo.set_servo_pwm(str(ch), ang)
        except Exception:
            log.warning("Servo not available")
            return
    setter(0, config.PAN_CENTER)
    setter(1, config.TILT_DOWN)
    log.info(f"Servo: pan={config.PAN_CENTER}, tilt={config.TILT_DOWN}")


def get_pose(camera, detector, localizer):
    import cv2
    frame = camera.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    detections = detector.detect(gray)
    if not detections:
        return None
    return localizer.compute_pose(detections)


def navigate(goal_x, goal_y, goal_yaw, goal_tag_id=None, test_only=False):
    camera = init_camera()
    init_servo()
    detector = AprilTagDetector()
    localizer = Localizer()
    motor = MotorAdapter()
    sensor = UltrasonicSensor()
    avoider = ObstacleAvoider(sensor, motor)

    dt = 1.0 / config.CONTROL_HZ
    path = None
    follower = None
    motors_running = False
    last_tag_time = None
    coasting = False

    warmup_poses = []
    warmup_target = config.MIN_POSE_READINGS

    log.info(f"Goal: ({goal_x:.4f}, {goal_y:.4f}) yaw={math.degrees(goal_yaw):.0f}")
    if goal_tag_id:
        log.info(f"  Goal tag: {goal_tag_id}")
    if sensor.available:
        log.info(f"Obstacle avoidance: ENABLED (threshold={config.OBSTACLE_THRESHOLD_CM:.0f}cm)")
    else:
        log.info("Obstacle avoidance: DISABLED")
    if test_only:
        log.info("TEST MODE -- detecting tags, not driving")

    try:
        while True:
            # === OBSTACLE CHECK #1 — before camera ===
            if not test_only and motors_running and sensor.available:
                if _check_and_avoid(avoider, motor):
                    motors_running = False
                    coasting = False
                    continue

            # === PERCEPTION (slow — ~200ms) ===
            pose = get_pose(camera, detector, localizer)
            now = time.monotonic()

            # === OBSTACLE CHECK #2 — after camera ===
            if not test_only and motors_running and sensor.available:
                if _check_and_avoid(avoider, motor):
                    motors_running = False
                    coasting = False
                    continue

            if pose is None:
                if last_tag_time is None:
                    if test_only:
                        print("  (no tags)")
                    time.sleep(dt)
                    continue

                time_without_tags = now - last_tag_time

                if motors_running and time_without_tags < TAG_LOSS_COAST_S:
                    if not coasting:
                        log.info("Tags lost -- coasting across gap")
                        coasting = True
                    time.sleep(dt)
                    continue
                elif motors_running and time_without_tags < TAG_LOSS_STOP_S:
                    crawl = max(config.MOTOR_DEADBAND + 3, config.VMAX * 0.25)
                    motor.set_motors(crawl, True, crawl, True)
                    time.sleep(dt)
                    continue
                else:
                    if motors_running:
                        motor.stop()
                        motors_running = False
                        coasting = False
                        log.warning("Tags lost too long -- stopped")
                    time.sleep(dt)
                    continue

            last_tag_time = now
            coasting = False

            bx = pose["board_x"]
            by = pose["board_y"]
            byaw_deg = pose["board_yaw"]
            byaw_rad = math.radians(byaw_deg)
            tag = pose["closest_tag_id"]

            if test_only:
                us_dist = sensor.measure_cm() if sensor.available else -1
                print(f"  Pose: ({bx:.4f}, {by:.4f})  yaw={byaw_deg:.1f}"
                      f"  nearest=tag{tag}  conf={pose['confidence']:.2f}"
                      f"  tags={pose['num_tags']}  us={us_dist:.0f}cm")
                time.sleep(dt)
                continue

            # Warmup
            if path is None and len(warmup_poses) < warmup_target:
                warmup_poses.append((bx, by, byaw_rad))
                log.info(f"Warmup {len(warmup_poses)}/{warmup_target}: "
                         f"({bx:.4f}, {by:.4f}) yaw={byaw_deg:.1f}")
                time.sleep(dt)
                continue

            # Plan path (once)
            if path is None:
                avg_x = sum(p[0] for p in warmup_poses) / len(warmup_poses)
                avg_y = sum(p[1] for p in warmup_poses) / len(warmup_poses)
                avg_sin = sum(math.sin(p[2]) for p in warmup_poses) / len(warmup_poses)
                avg_cos = sum(math.cos(p[2]) for p in warmup_poses) / len(warmup_poses)
                avg_yaw = math.atan2(avg_sin, avg_cos)

                log.info(f"Start: ({avg_x:.4f}, {avg_y:.4f}) yaw={math.degrees(avg_yaw):.1f}")

                start = {"x": avg_x, "y": avg_y, "yaw": avg_yaw}
                goal  = {"x": goal_x, "y": goal_y, "yaw": goal_yaw}

                raw_path = plan_path(start, goal)
                if raw_path is None:
                    log.error("No path found!")
                    motor.stop()
                    return False

                path = raw_path
                log.info(f"Path: {len(path)} waypoints")
                for i, wp in enumerate(path):
                    log.info(f"  WP{i}: ({wp['x']:.4f}, {wp['y']:.4f})"
                             f"  yaw={math.degrees(wp['yaw']):.0f}")

                follower = PathFollower(
                    path=path,
                    vmax=config.VMAX,
                    turn_speed=config.TURN_SPEED,
                    pos_tol_m=config.POS_TOLERANCE_M,
                    goal_tol_m=config.GOAL_TOLERANCE_M,
                    yaw_tol_rad=config.YAW_TOLERANCE_RAD,
                    deadband=config.MOTOR_DEADBAND,
                    goal_tag_id=goal_tag_id,
                )

            # Follow path
            ls, lf, rs, rf = follower.step(
                bx, by, byaw_rad,
                dt=dt,
                closest_tag_id=tag,
            )

            if follower.done:
                motor.stop()
                motors_running = False
                if follower.success:
                    log.info("=== GOAL REACHED ===")
                    return True
                else:
                    log.warning("Path follower stopped (not success)")
                    return False

            # Drive
            any_motion = (ls > 0 or rs > 0)
            if any_motion:
                motor.set_motors(ls, lf, rs, rf)
                motors_running = True
            else:
                motor.stop()
                motors_running = False

            # Status
            dist = math.sqrt((goal_x - bx)**2 + (goal_y - by)**2)
            herr = math.degrees(_wrap(math.atan2(goal_x - bx, -(goal_y - by)) - byaw_rad))
            log.info(
                f"WP{follower.current_index}/{follower.total_waypoints}"
                f"  pos=({bx:.3f},{by:.3f})  yaw={byaw_deg:.0f}"
                f"  dist={dist:.3f}m  herr={herr:.0f}"
                f"  tag={tag}"
                f"  motors=L{ls:.0f}{'F' if lf else 'R'}"
                f" R{rs:.0f}{'F' if rf else 'R'}"
            )

            time.sleep(dt)

    except KeyboardInterrupt:
        log.info("Interrupted by user")
    finally:
        motor.stop()
        sensor.cleanup()
        try:
            camera.stop()
        except Exception:
            pass

    return False


def main():
    parser = argparse.ArgumentParser(
        description="Navigate the Freenove car to an AprilTag position",
    )
    parser.add_argument("tag_id", nargs="?", type=int, help="Target tag ID")
    parser.add_argument("yaw", nargs="?", type=float, default=None,
                        help="Goal heading in degrees")
    parser.add_argument("--xy", nargs="+", type=float, metavar="V",
                        help="Goal as X Y [YAW_DEG]")
    parser.add_argument("--list", action="store_true", help="List all tag positions")
    parser.add_argument("--test", action="store_true", help="Detect only (no driving)")
    args = parser.parse_args()

    if args.list:
        print(f"\n{len(TAG_LAYOUT)} tags on board"
              f" ({config.BOARD_WIDTH_M:.3f} x {config.BOARD_HEIGHT_M:.3f} m)\n")
        for tid in sorted(TAG_LAYOUT):
            x, y, yaw = TAG_LAYOUT[tid]
            print(f"  Tag {tid:>3d}:  x={x:.4f}  y={y:.4f}  m")
        return 0

    if args.test:
        navigate(0, 0, 0, test_only=True)
        return 0

    if args.xy is not None:
        if len(args.xy) < 2:
            parser.error("--xy needs at least X Y")
        gx, gy = args.xy[0], args.xy[1]
        gyaw = math.radians(args.xy[2]) if len(args.xy) > 2 else 0.0
        ok = navigate(gx, gy, gyaw)
        return 0 if ok else 1

    if args.tag_id is None:
        parser.error("Provide a TAG_ID, --xy, --list, or --test")

    tag_map = TagMap()
    target = tag_map.get_pose(args.tag_id)
    if target is None:
        print(f"Tag {args.tag_id} not found! Use --list to see valid IDs.")
        return 1

    tx, ty, tyaw = target
    if args.yaw is not None:
        goal_yaw = math.radians(args.yaw)
    else:
        goal_yaw = tyaw

    ok = navigate(tx, ty, goal_yaw, goal_tag_id=args.tag_id)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
