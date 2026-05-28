"""Finite state machine driving autonomous behavior."""
from __future__ import annotations
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional
import config
from environment import EnvironmentMemory
from vision import VisionResult


class CarState(str, Enum):
    """High-level states for navigation."""
    IDLE = "IDLE"
    FORWARD_DRIVE = "FORWARD_DRIVE"
    OBSTACLE_DETECTED = "OBSTACLE_DETECTED"
    AVOIDANCE = "AVOIDANCE"
    STOPPED = "STOPPED"
    TAG_ACTION = "TAG_ACTION"


@dataclass
class SensorSnapshot:
    """Unified snapshot from sensors and vision."""
    distance_cm: Optional[float]
    vision: VisionResult


class StateMachine:
    """State machine coordinating motion logic and transitions."""

    def __init__(self, memory: EnvironmentMemory) -> None:
        self.state = CarState.IDLE
        self._state_enter_time = time.time()
        self._memory = memory
        self._avoid_direction: Optional[str] = None
        self._tag_action: Optional[str] = None

    def transition(self, new_state: CarState) -> None:
        if new_state != self.state:
            self.state = new_state
            self._state_enter_time = time.time()

    def state_elapsed(self) -> float:
        return time.time() - self._state_enter_time

    def step(self, sensors: SensorSnapshot) -> CarState:
        distance = sensors.distance_cm
        vision = sensors.vision

        obstacle_near = distance is not None and distance < config.SAFE_DISTANCE_CM
        emergency = distance is not None and distance < config.EMERGENCY_DISTANCE_CM
        vision_obstacle = vision.obstacle_detected

        if emergency:
            self.transition(CarState.STOPPED)
            return self.state

        # AprilTag actions take priority over normal obstacle avoidance
        if vision.marker_action is not None:
            self._tag_action = vision.marker_action
            self.transition(CarState.TAG_ACTION)
            return self.state

        if self.state == CarState.IDLE:
            if obstacle_near or vision_obstacle:
                self._memory.record_obstacle(vision.obstacle_zone)
                self.transition(CarState.OBSTACLE_DETECTED)
            else:
                self.transition(CarState.FORWARD_DRIVE)

        elif self.state == CarState.FORWARD_DRIVE:
            if obstacle_near or vision_obstacle:
                self._memory.record_obstacle(vision.obstacle_zone)
                self.transition(CarState.OBSTACLE_DETECTED)

        elif self.state == CarState.OBSTACLE_DETECTED:
            self._avoid_direction = self._memory.choose_avoidance_direction()
            self.transition(CarState.AVOIDANCE)

        elif self.state == CarState.AVOIDANCE:
            if self.state_elapsed() >= config.TURN_DURATION_SEC:
                if obstacle_near or vision_obstacle:
                    self.transition(CarState.OBSTACLE_DETECTED)
                else:
                    self.transition(CarState.FORWARD_DRIVE)

        elif self.state == CarState.TAG_ACTION:
            # Hold the tag action for the turn duration then re-evaluate
            if self.state_elapsed() >= config.TURN_DURATION_SEC:
                self._tag_action = None
                self.transition(CarState.IDLE)

        elif self.state == CarState.STOPPED:
            if not emergency and not obstacle_near:
                self.transition(CarState.IDLE)

        return self.state

    @property
    def avoidance_direction(self) -> Optional[str]:
        return self._avoid_direction

    @property
    def tag_action(self) -> Optional[str]:
        return self._tag_action