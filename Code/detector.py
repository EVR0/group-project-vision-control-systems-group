"""
AprilTag detector with 6-DOF pose estimation.

Supports dt_apriltags or pyapriltags — whichever is installed.
Camera intrinsics are computed from the MAIN STREAM resolution (640×480),
NOT the raw sensor resolution.
"""

import cv2
import numpy as np
import logging
from typing import List
import config

log = logging.getLogger(__name__)

# Try both libraries
_DETECTOR_LIB = None
try:
    from dt_apriltags import Detector as _DtDetector
    _DETECTOR_LIB = "dt_apriltags"
except ImportError:
    pass

if _DETECTOR_LIB is None:
    try:
        from pyapriltags import Detector as _PyDetector
        _DETECTOR_LIB = "pyapriltags"
    except ImportError:
        pass

if _DETECTOR_LIB is None:
    log.critical(
        "No AprilTag library found! Install one:\n"
        "  sudo pip install dt-apriltags --break-system-packages\n"
        "  OR\n"
        "  sudo pip install pyapriltags --break-system-packages"
    )


class Detection:
    """Single AprilTag detection with pose."""

    def __init__(self, tag_id: int, corners: np.ndarray,
                 tvec: np.ndarray, R: np.ndarray):
        self.tag_id = tag_id
        self.corners = corners       # (1, 4, 2) pixel coords
        self.tvec = tvec             # translation vector (camera frame)
        self.R = R                   # 3×3 rotation matrix (tag-in-camera)
        self.decision_margin = 0.0

    @property
    def rvec(self) -> np.ndarray:
        """Rodrigues rotation vector (for compatibility)."""
        rvec, _ = cv2.Rodrigues(self.R)
        return rvec.flatten()

    @property
    def rotation_matrix(self) -> np.ndarray:
        return self.R


class AprilTagDetector:
    """Detects AprilTags and estimates pose relative to camera."""

    def __init__(self):
        self.tag_family = config.TAG_FAMILY
        self.tag_size = config.TAG_SIZE_M * config.EFFECTIVE_TAG_SCALE

        # Intrinsics for the MAIN STREAM — must match capture resolution
        w, h = config.CAMERA_RESOLUTION   # (640, 480)

        # Pi Camera Module 3 (IMX708) approximate HFOV ≈ 66°.
        # focal = w / (2 * tan(HFOV/2))  ≈  640 / (2 * tan(33°)) ≈ 493
        # Using 0.77 * width as a reasonable estimate without calibration.
        focal = w * 0.77
        self.camera_params = [focal, focal, w / 2.0, h / 2.0]

        self.camera_matrix = np.array([
            [focal, 0, w / 2.0],
            [0, focal, h / 2.0],
            [0, 0, 1],
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        log.info(f"Camera intrinsics: fx={focal:.1f}  cx={w/2:.0f}  cy={h/2:.0f}"
                 f"  (for {w}×{h} stream)")

        # Init detector
        self._detector = None
        if _DETECTOR_LIB == "dt_apriltags":
            self._detector = _DtDetector(
                families=self.tag_family,
                nthreads=2,
                quad_decimate=1.0,
                quad_sigma=0.0,
                decode_sharpening=0.25,
            )
            log.info(f"Using dt_apriltags ({self.tag_family})")
        elif _DETECTOR_LIB == "pyapriltags":
            self._detector = _PyDetector(
                families=self.tag_family,
                nthreads=2,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0,
            )
            log.info(f"Using pyapriltags ({self.tag_family})")

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """Detect tags in a frame.  Accepts BGR or grayscale."""
        if self._detector is None:
            return []

        gray = (cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if len(frame.shape) == 3 else frame)

        results = self._detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )

        detections = []
        for r in results:
            tvec = r.pose_t.flatten()
            # pose_R is already a 3×3 rotation matrix — store it directly
            R = np.array(r.pose_R, dtype=np.float64).reshape(3, 3)
            corners = r.corners.reshape(1, 4, 2)

            det = Detection(int(r.tag_id), corners, tvec, R)
            if hasattr(r, "decision_margin"):
                det.decision_margin = float(r.decision_margin)
            detections.append(det)

        return detections
