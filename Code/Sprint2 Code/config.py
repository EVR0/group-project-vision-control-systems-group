"""Central configuration for the Freenove 4WD autonomous car."""
# Camera settings (Picamera2)
CAMERA_RESOLUTION = (640, 480)
CAMERA_FPS = 30
# Vision settings
VISION_ROI_Y_START = 0.55
VISION_MIN_CONTOUR_AREA = 1800
VISION_BLUR_KERNEL = (5, 5)
VISION_CANNY_THRESHOLDS = (60, 140)
USE_ARUCO_MARKERS = False
ARUCO_DICTIONARY = "DICT_4X4_50"
# Ultrasonic safety thresholds (centimeters)
SAFE_DISTANCE_CM = 40
EMERGENCY_DISTANCE_CM = 22
# Motor control
FORWARD_SPEED = 4095
TURN_SPEED = 4095
TURN_DURATION_SEC = 2.0
REVERSE_DURATION_SEC = 0.25
LEFT_TRIM = 0.70
# Servo angles (degrees)
PAN_CENTER = 90
TILT_CENTER = 90
# Main loop control
LOOP_HZ = 12
# Feedback patterns
BUZZER_WARNING_BEEP_MS = 120
# Optional LED colors (RGB tuples)
LED_COLOR_IDLE = (0, 0, 255)
LED_COLOR_FORWARD = (0, 255, 0)
LED_COLOR_OBSTACLE = (255, 80, 0)
LED_COLOR_AVOIDANCE = (255, 0, 0)
LED_COLOR_STOPPED = (255, 0, 255)

# AprilTag settings
USE_APRILTAGS = True
APRILTAG_FAMILY = "tag36h11"
APRILTAG_SIZE_CM = 6.0  # physical size of your printed tags

# Tag ID to action mapping
# Customize these to whatever tag IDs you print
APRILTAG_ACTIONS = {
    0: "stop",
    1: "turn_left",
    2: "turn_right",
    3: "slow_down",
    4: "reverse",
}

# Distance at which to react to a tag (in pixels, based on tag perimeter)
APRILTAG_REACT_PERIMETER = 200