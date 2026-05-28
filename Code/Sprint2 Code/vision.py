"""Vision processing with AprilTag detection for the Freenove 4WD car."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional
import numpy as np
import config


@dataclass
class VisionResult:
    """Container for vision processing results."""
    obstacle_detected: bool = False
    obstacle_zone: Optional[str] = None
    marker_id: Optional[int] = None
    marker_action: Optional[str] = None
    marker_distance: Optional[float] = None
    marker_center: Optional[tuple] = None


class VisionProcessor:
    """Handles camera capture, obstacle detection, and AprilTag detection."""

    def __init__(self) -> None:
        from picamera2 import Picamera2  # type: ignore
        import cv2

        self._cv2 = cv2
        self._camera = Picamera2()
        cam_config = self._camera.create_preview_configuration(
            main={"size": config.CAMERA_RESOLUTION, "format": "RGB888"}
        )
        self._camera.configure(cam_config)
        self._camera.start()

        # Set up AprilTag detector if enabled
        self._apriltag_detector = None
        if config.USE_APRILTAGS:
            try:
                from dt_apriltags import Detector  # type: ignore
                self._apriltag_detector = Detector(
                    families=config.APRILTAG_FAMILY,
                    nthreads=2,
                    quad_decimate=2.0,
                    quad_sigma=0.0,
                    decode_sharpening=0.25,
                )
                print("AprilTag detector initialized successfully.")
            except ImportError:
                print("dt_apriltags not installed. Run: sudo pip install dt-apriltags --break-system-packages")
                self._apriltag_detector = None

    def capture_frame(self) -> np.ndarray:
        """Grab a single frame from the camera."""
        return self._camera.capture_array()

    def detect_obstacles(self, frame: np.ndarray) -> VisionResult:
        """Run obstacle detection and AprilTag detection on a frame."""
        result = VisionResult()

        # Standard obstacle detection using edge detection
        h, w = frame.shape[:2]
        roi_y = int(h * config.VISION_ROI_Y_START)
        roi = frame[roi_y:h, 0:w]

        gray = self._cv2.cvtColor(roi, self._cv2.COLOR_RGB2GRAY)
        blurred = self._cv2.GaussianBlur(gray, config.VISION_BLUR_KERNEL, 0)
        edges = self._cv2.Canny(
            blurred,
            config.VISION_CANNY_THRESHOLDS[0],
            config.VISION_CANNY_THRESHOLDS[1],
        )

        contours, _ = self._cv2.findContours(edges, self._cv2.RETR_EXTERNAL, self._cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if self._cv2.contourArea(c) > config.VISION_MIN_CONTOUR_AREA:
                x, _, cw, _ = self._cv2.boundingRect(c)
                center_x = x + cw // 2
                if center_x < w // 3:
                    result.obstacle_zone = "left"
                elif center_x > 2 * w // 3:
                    result.obstacle_zone = "right"
                else:
                    result.obstacle_zone = "center"
                result.obstacle_detected = True
                break

        # AprilTag detection
        if self._apriltag_detector is not None:
            self._detect_apriltags(frame, result)

        return result

    def _detect_apriltags(self, frame: np.ndarray, result: VisionResult) -> None:
        """Detect AprilTags in the frame and update the result."""
        gray_full = self._cv2.cvtColor(frame, self._cv2.COLOR_RGB2GRAY)
        detections = self._apriltag_detector.detect(gray_full)

        if not detections:
            return

        # Pick the closest tag (largest perimeter)
        best = None
        best_perimeter = 0

        for det in detections:
            corners = det.corners
            perimeter = 0
            for i in range(4):
                dx = corners[(i + 1) % 4][0] - corners[i][0]
                dy = corners[(i + 1) % 4][1] - corners[i][1]
                perimeter += (dx ** 2 + dy ** 2) ** 0.5

            if perimeter > best_perimeter:
                best_perimeter = perimeter
                best = det

        if best is None:
            return

        result.marker_id = best.tag_id
        result.marker_center = (int(best.center[0]), int(best.center[1]))
        result.marker_distance = best_perimeter

        # Only react if the tag is close enough
        if best_perimeter >= config.APRILTAG_REACT_PERIMETER:
            action = config.APRILTAG_ACTIONS.get(best.tag_id)
            if action:
                result.marker_action = action

    def close(self) -> None:
        """Release camera resources."""
        try:
            self._camera.stop()
        except Exception:
            pass