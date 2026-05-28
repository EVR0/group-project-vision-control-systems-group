"""
Path-following controller — proportional steering (no in-place turns).

Always drives forward, adjusts left/right wheel speeds to steer toward
the goal. This keeps the camera pointed at the floor so tags stay visible.

If heading error is very large (> 90 deg), does a brief reverse to
reposition rather than spinning in place.

Output per tick: (left_speed, left_fwd, right_speed, right_fwd)
  Speeds are 0-100 (motor scale).  fwd = True means that side goes forward.
"""

import math
import logging
from typing import List, Dict, Optional

log = logging.getLogger(__name__)


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class PathFollower:
    """
    Proportional-steering path follower for differential-drive robots.
    No in-place turns — always drives forward with differential steering.
    """

    def __init__(self, path: List[Dict[str, float]],
                 vmax=50.0, turn_speed=40.0,
                 pos_tol_m=0.12, goal_tol_m=0.04,
                 yaw_tol_rad=0.20, deadband=8.0,
                 goal_tag_id: Optional[int] = None,
                 **kwargs):
        self.path = path
        self.vmax = vmax
        self.turn_speed = turn_speed
        self.pos_tol_m = pos_tol_m
        self.goal_tol_m = goal_tol_m
        self.yaw_tol_rad = yaw_tol_rad
        self.deadband = deadband
        self.goal_tag_id = goal_tag_id

        self._index = 0
        self._phase = "DRIVE"

        self.done = False
        self.success = False

    @property
    def current_index(self) -> int:
        return self._index

    @property
    def total_waypoints(self) -> int:
        return len(self.path)

    def step(self, cur_x, cur_y, cur_yaw, dt=0.1,
             closest_tag_id=None) -> tuple:
        """
        One control tick.
        Returns (left_speed, left_fwd, right_speed, right_fwd).
        """
        if self.done or self._index >= len(self.path):
            self.done = True
            self.success = True
            return (0, True, 0, True)

        # Tag-match stop
        if (self.goal_tag_id is not None
                and closest_tag_id == self.goal_tag_id
                and self._index == len(self.path) - 1):
            self.done = True
            self.success = True
            log.info(f"Stopped on goal tag {self.goal_tag_id}")
            return (0, True, 0, True)

        wp = self.path[self._index]
        gx, gy, g_yaw = wp["x"], wp["y"], wp["yaw"]
        is_last = self._index == len(self.path) - 1

        dx = gx - cur_x
        dy = gy - cur_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Check if waypoint reached
        tol = self.goal_tol_m if is_last else self.pos_tol_m
        if dist <= tol:
            if is_last:
                self.done = True
                self.success = True
                log.info("Goal position reached")
                return (0, True, 0, True)
            else:
                self._index += 1
                log.info(f"Advanced to WP{self._index}")
                return (0, True, 0, True)

        # Desired heading toward waypoint
        desired_heading = math.atan2(dx, -dy)
        heading_error = _wrap(desired_heading - cur_yaw)
        abs_err = abs(heading_error)

        # Base speed: proportional to distance, minimum 50% vmax
        base_speed = max(self.vmax * 0.50,
                         min(self.vmax, self.vmax * (dist / 0.3)))

        # Slow down when close to goal
        if is_last and dist < 0.10:
            base_speed = max(self.vmax * 0.35, self.deadband + 5)

        # --- Steering logic ---

        if abs_err > math.radians(120):
            # Way off — goal is behind us. Do a tight pivot-turn:
            # one wheel forward, one stopped. This turns while keeping
            # some forward motion so camera stays on tags.
            spd = max(self.turn_speed * 0.7, self.deadband + 2)
            if heading_error > 0:
                # Need to turn right: left forward, right stopped
                return (spd, True, 0, True)
            else:
                # Need to turn left: right forward, left stopped
                return (0, True, spd, True)

        if abs_err > math.radians(60):
            # Large error — sharp differential turn while moving forward.
            # Fast wheel at turn_speed, slow wheel at deadband.
            fast = max(self.turn_speed, self.deadband + 5)
            slow = self.deadband
            if heading_error > 0:
                return (fast, True, slow, True)
            else:
                return (slow, True, fast, True)

        # Normal driving with proportional steering correction
        # steer_factor: 0 at 0 error, 1.0 at 60 degrees error
        steer_factor = min(abs_err / math.radians(60), 1.0)

        # Max correction: reduce one side by up to 70% of base speed
        correction = steer_factor * base_speed * 0.70

        if heading_error > 0:
            # Turn right: left faster, right slower
            ls = min(self.vmax, base_speed + correction * 0.3)
            rs = max(self.deadband, base_speed - correction)
        else:
            # Turn left: right faster, left slower
            ls = max(self.deadband, base_speed - correction)
            rs = min(self.vmax, base_speed + correction * 0.3)

        return (ls, True, rs, True)
