"""
Localizer — computes vehicle position from AprilTag detections.

Heading extraction uses the camera X-axis (R_tc[:,0]) which lies in the
floor plane and is stable for downward-looking cameras.

Includes filtering:
  - Single-tag detections with low margin are rejected for heading
  - Heading jumps > 90 deg from previous reading are rejected
"""

import math
import numpy as np
import logging
from typing import List, Optional, Dict

from detector import Detection
from tag_layout import TAG_LAYOUT
import config

log = logging.getLogger(__name__)

_SX = -1.0 if config.POSE_INVERT_X else 1.0
_SY = -1.0 if config.POSE_INVERT_Y else 1.0

# Minimum decision margin to trust a single-tag heading
_MIN_SINGLE_TAG_MARGIN = 40.0

# Maximum allowed heading jump between consecutive readings (degrees)
_MAX_YAW_JUMP_DEG = 90.0


def _wrap_deg(a):
    """Wrap angle to -180..+180 degrees."""
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


class Localizer:

    def __init__(self):
        self._prev_yaw = None  # previous valid heading (degrees, 0-360)

    def compute_pose(self, detections: List[Detection]) -> Optional[Dict]:
        if not detections:
            return None

        positions_x = []
        positions_y = []
        headings_sin = []
        headings_cos = []

        closest_tag_id = None
        min_dist = float("inf")

        valid_detections = [d for d in detections if int(d.tag_id) in TAG_LAYOUT]
        if not valid_detections:
            return None

        num_valid = len(valid_detections)

        for det in valid_detections:
            tag_id = int(det.tag_id)
            tag_x, tag_y, _ = TAG_LAYOUT[tag_id]

            R_ct = det.R
            t = det.tvec.flatten()

            # Camera position in tag frame
            cam_in_tag = -R_ct.T @ t

            # Map to board frame
            robot_x = tag_x + _SX * cam_in_tag[0]
            robot_y = tag_y + _SY * cam_in_tag[1]

            positions_x.append(robot_x)
            positions_y.append(robot_y)

            # --- Heading from camera X-axis ---
            R_tc = R_ct.T
            cam_right_x = R_tc[0, 0]
            cam_right_y = R_tc[1, 0]

            right_angle = math.atan2(_SX * cam_right_x, _SY * -cam_right_y)
            heading = math.pi / 2.0 - right_angle

            # For single-tag detections, only use heading if margin is high enough
            margin = getattr(det, "decision_margin", 50.0)
            if num_valid == 1 and margin < _MIN_SINGLE_TAG_MARGIN:
                # Still use position, but skip this heading (unreliable)
                pass
            else:
                headings_sin.append(math.sin(heading))
                headings_cos.append(math.cos(heading))

            # Track closest tag
            dist = float(np.linalg.norm(t))
            if dist < min_dist:
                min_dist = dist
                closest_tag_id = tag_id

        # Position: always average all valid detections
        n = len(positions_x)
        avg_x = sum(positions_x) / n
        avg_y = sum(positions_y) / n

        # Heading: use filtered headings, fall back to previous if none
        if headings_sin:
            avg_heading = math.degrees(math.atan2(
                sum(headings_sin) / len(headings_sin),
                sum(headings_cos) / len(headings_cos),
            )) % 360.0

            # Continuity check: reject large jumps from previous reading
            if self._prev_yaw is not None:
                jump = abs(_wrap_deg(avg_heading - self._prev_yaw))
                if jump > _MAX_YAW_JUMP_DEG:
                    log.debug(f"Heading jump {jump:.0f} deg rejected "
                              f"({self._prev_yaw:.0f} -> {avg_heading:.0f})")
                    avg_heading = self._prev_yaw
                else:
                    self._prev_yaw = avg_heading
            else:
                self._prev_yaw = avg_heading
        else:
            # No reliable heading this frame — use previous
            if self._prev_yaw is not None:
                avg_heading = self._prev_yaw
            else:
                # No heading at all yet — can't localize heading
                avg_heading = 0.0

        # Closest tag by board distance
        best_tid = closest_tag_id
        best_dsq = float("inf")
        for tid, (tx, ty, _) in TAG_LAYOUT.items():
            dsq = (avg_x - tx) ** 2 + (avg_y - ty) ** 2
            if dsq < best_dsq:
                best_dsq = dsq
                best_tid = tid

        # Confidence
        margins = [getattr(d, "decision_margin", 50.0) for d in valid_detections]
        confidence = min(1.0, (sum(margins) / len(margins)) / 80.0) if margins else 0.0

        return {
            "board_x": float(avg_x),
            "board_y": float(avg_y),
            "board_yaw": float(avg_heading),
            "closest_tag_id": best_tid,
            "confidence": float(confidence),
            "is_valid": True,
            "num_tags": n,
            "visible_tag_ids": [int(d.tag_id) for d in valid_detections],
        }
