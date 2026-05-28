"""
Configuration for Freenove 4WD AprilTag Navigation.

Hardware: Raspberry Pi Camera Module 3 (IMX708), Freenove 4WD Smart Car Kit.
Board: 8x8 ft (2.4384m x 2.4384m) with 1098 AprilTags.
"""

# ==========================================================================
# Camera Settings — Pi Camera Module 3 (IMX708)
# ==========================================================================
CAMERA_RESOLUTION = (640, 480)
CAMERA_HEIGHT_M = 0.12
CAMERA_PITCH_DEG = 45.0
CAMERA_X_M = 0.0
CAMERA_Y_M = 0.02
CAMERA_HFLIP = True
CAMERA_VFLIP = True
CAMERA_FOCUS_MODE = "manual"
CAMERA_LENS_POSITION = 8.0

# ==========================================================================
# AprilTag Detection
# ==========================================================================
TAG_FAMILY = "tagStandard41h12"
TAG_SIZE_M = 0.040   # 40 mm (4 cm)
EFFECTIVE_TAG_SCALE = 7.0 / 9.0

# ==========================================================================
# Board / Tag Layout
# ==========================================================================
BOARD_WIDTH_M = 2.4384
BOARD_HEIGHT_M = 2.4384

# ==========================================================================
# Motor Control (Freenove)
# ==========================================================================
MAX_MOTOR_SPEED = 4095
LEFT_TRIM = 0.95
USE_ALL_WHEELS = False

# ==========================================================================
# Navigation — Path Planner (Hybrid A*)
# ==========================================================================
GRID_RESOLUTION_M = 0.02
ROBOT_RADIUS_M = 0.0
TURN_PENALTY = 2.0
MAX_PLANNER_ITERATIONS = 200_000

# ==========================================================================
# Navigation — Path Follower
# ==========================================================================
VMAX = 38.0
TURN_SPEED = 30.0
POS_TOLERANCE_M = 0.05
GOAL_TOLERANCE_M = 0.04
YAW_TOLERANCE_RAD = 0.70
MOTOR_DEADBAND = 8.0
CONTROL_HZ = 10.0
MIN_POSE_READINGS = 3

# ==========================================================================
# Servo (pan-tilt camera mount)
# ==========================================================================
PAN_CENTER = 90
TILT_DOWN = 60

# ==========================================================================
# Coordinate convention
# ==========================================================================
POSE_INVERT_X = False
POSE_INVERT_Y = True

# ==========================================================================
# Obstacle Avoidance (Ultrasonic HC-SR04)
# ==========================================================================
ULTRASONIC_TRIG_PIN = 27
ULTRASONIC_ECHO_PIN = 22
OBSTACLE_THRESHOLD_CM = 40.0
AVOID_TURN_SPEED = 50.0
AVOID_DRIVE_SPEED = 50.0
AVOID_TURN_TIME_S = 0.40
AVOID_DRIVE_TIME_S = 0.50
AVOID_RETURN_TURN_S = 0.50
