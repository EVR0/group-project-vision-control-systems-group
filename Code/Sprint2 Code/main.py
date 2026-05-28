from __future__ import annotations

import sys
import time

import config
from environment import EnvironmentMemory
from feedback import FeedbackManager
from motor_controller import MotorController
from sensors import UltrasonicSensor
from servo_controller import ServoController
from state_machine import CarState, SensorSnapshot, StateMachine
from vision import VisionProcessor


class AutonomousCar:
    """High-level system that wires hardware to the navigation logic."""

    def __init__(self) -> None:
        self.motor = MotorController()
        self.servo = ServoController()
        self.ultrasonic = UltrasonicSensor()
        self.vision = VisionProcessor()
        self.feedback = FeedbackManager()
        self.memory = EnvironmentMemory()
        self.state_machine = StateMachine(self.memory)

        self.servo.center(config.PAN_CENTER, config.TILT_CENTER)

    def shutdown(self) -> None:
        """Safely stop all hardware."""
        self.motor.stop()
        self.feedback.shutdown()
        try:
            self.vision.close()
        except Exception:
            pass

    def update_feedback(self, state: CarState) -> None:
        """Set LEDs and buzzer based on current state."""
        if state == CarState.IDLE:
            self.feedback.set_led_color(config.LED_COLOR_IDLE)
        elif state == CarState.FORWARD_DRIVE:
            self.feedback.set_led_color(config.LED_COLOR_FORWARD)
        elif state == CarState.OBSTACLE_DETECTED:
            self.feedback.set_led_color(config.LED_COLOR_OBSTACLE)
            self.feedback.warning_beep()
        elif state == CarState.AVOIDANCE:
            self.feedback.set_led_color(config.LED_COLOR_AVOIDANCE)
        elif state == CarState.STOPPED:
            self.feedback.set_led_color(config.LED_COLOR_STOPPED)
            self.feedback.warning_beep()
        elif state == CarState.TAG_ACTION:
            self.feedback.set_led_color(config.LED_COLOR_OBSTACLE)

    def act_on_state(self, state: CarState) -> None:
        """Translate state into motor commands."""
        if state in (CarState.IDLE, CarState.OBSTACLE_DETECTED, CarState.STOPPED):
            self.motor.stop()
            return

        if state == CarState.FORWARD_DRIVE:
            self.motor.forward(config.FORWARD_SPEED)
            return

        if state == CarState.AVOIDANCE:
            direction = self.state_machine.avoidance_direction
            if direction == "left":
                self.motor.turn_left(config.TURN_SPEED)
            else:
                self.motor.turn_right(config.TURN_SPEED)
            return

        if state == CarState.TAG_ACTION:
            action = self.state_machine.tag_action
            if action == "stop":
                self.motor.stop()
            elif action == "turn_left":
                self.motor.turn_left(config.TURN_SPEED)
            elif action == "turn_right":
                self.motor.turn_right(config.TURN_SPEED)
            elif action == "slow_down":
                self.motor.forward(config.FORWARD_SPEED // 3)
            elif action == "reverse":
                self.motor.backward(config.TURN_SPEED)


def main() -> int:
    """Main control loop with safe initialization and shutdown."""
    try:
        car = AutonomousCar()
    except Exception as exc:
        print(f"Hardware initialization failed: {exc}")
        return 1

    loop_delay = 1.0 / config.LOOP_HZ

    try:
        while True:
            frame = car.vision.capture_frame()
            vision_result = car.vision.detect_obstacles(frame)
            distance = car.ultrasonic.get_distance_cm()

            snapshot = SensorSnapshot(distance_cm=distance, vision=vision_result)
            state = car.state_machine.step(snapshot)

            car.update_feedback(state)
            car.act_on_state(state)

            print(
                "State={state} Dist={dist} Vision={vision} Zone={zone} Marker={marker} Action={action}".format(
                    state=state.value,
                    dist=f"{distance:.1f}cm" if distance is not None else "N/A",
                    vision="Y" if vision_result.obstacle_detected else "N",
                    zone=vision_result.obstacle_zone or "-",
                    marker=vision_result.marker_id if vision_result.marker_id is not None else "-",
                    action=vision_result.marker_action or "-",
                )
            )

            time.sleep(loop_delay)

    except KeyboardInterrupt:
        print("\nShutdown requested by user.")
    finally:
        car.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())