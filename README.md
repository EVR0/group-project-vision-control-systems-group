# Team 2 Final Project
# Smart Car Redesign and Vision-Based Autonomous Navigation


Course: Artificial Intelligence Systems Engineering Design I

Instructor: Dr. Shaimaa Ali

Date: April 3, 2026

Group: Team 2

Team members:
- Evan Romano (251327329)
- Joseph Toma (251283541)
- Mohammed Alamen Qassab (251285296)

## Project Summary

For this project, we redesigned the Freenove 4WD Smart Car to reduce mechanical complexity and improve autonomous performance. The original platform was modified to use front-wheel tank drive, a fixed camera mount, and a fixed ultrasonic mount. On the software side, we implemented AprilTag-based localization and goal navigation, with obstacle avoidance always taking priority.

Our final system combines perception, control, and actuation in one loop:
- Camera-based AprilTag detection and pose estimation
- Board-frame localization
- Path planning and path following
- Ultrasonic obstacle detection and avoidance
- Differential motor control

## Software Layout

The final code used for this project is in the Code folder.

Main runtime files include:
- Code/main.py
- Code/config.py
- Code/detector.py
- Code/localization.py
- Code/pathfinder.py
- Code/controller.py
- Code/motor_adapter.py
- Code/ultrasonic.py
- Code/obstacle_avoider.py

Testing scripts include:
- Code/test_detect.py
- Code/test_motors.py
- Code/test_turn.py
- Code/test_ultrasonic.py
- Code/test_ultrasonic_driving.py

An earlier sprint implementation is included in Code/Sprint2 Code.

## How We Run It

From the repository root, go to the code folder:
cd Code

List available tags:
python3 main.py --list

Run detection-only mode:
python3 main.py --test

Navigate to a target tag:
python3 main.py <TAG_ID>

Navigate to explicit coordinates:
python3 main.py --xy <X_M> <Y_M> [YAW_DEG]

Run subsystem tests:
python3 test_motors.py
python3 test_ultrasonic.py
python3 test_ultrasonic_driving.py
python3 test_detect.py
python3 test_turn.py

## Hardware and Dependencies

Hardware used:
- Raspberry Pi with CSI camera
- Freenove motor electronics stack
- HC-SR04 ultrasonic sensor

Ultrasonic pin mapping in our configuration:
- Trigger: GPIO 27
- Echo: GPIO 22

Main software dependencies:
- OpenCV
- NumPy
- Picamera2
- RPi.GPIO
- SMBus
- AprilTag library (dt-apriltags or pyapriltags)

## Other Project Assets

This repository also includes:
- CAD files and print files for mechanical redesign
- PCB design files and fabrication outputs
- Formal documentation (proposal, sprint reports, final report, presentation)
- Meeting minutes
- Photos and videos from testing and demonstrations

Note on PCB:
The PCB was designed during the project, but it was not integrated into the final robot due to time constraints.

## Final Notes

The complete design rationale, implementation details, testing process, and future work are documented in the final report in the Documentation folder.

