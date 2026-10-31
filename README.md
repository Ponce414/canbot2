# canbot2

An autonomous mecanum-drive robot with an SO-101 arm that finds cans, picks them up and
drops them in a bin. Raspberry Pi 5 (ROS 2 Jazzy) on top, an ESP32-S3 running the wheels
in real time, six STS3215 smart servos in the arm.

## Repository layout

| Path | Contents | Status |
| --- | --- | --- |
| `firmware/base_esp32/` | ESP32-S3 drive firmware, unit tests, bench tools | Milestone 1, ready for bench bring-up |
| `ros2_ws/src/` | ROS 2 packages (description, hardware, perception, behavior, bringup) | Upcoming |

## Milestones

Each milestone ends with a bench test that proves it before the next layer depends on it.

- [ ] **1. Drive firmware:** one wheel tracking its target under PID, then all four; safety
      paths tested; micro-ROS topics echo on the Pi. See `firmware/base_esp32/README.md`.
- [ ] **2. Robot description:** URDF/xacro with the base, wheels, SO-101 and sensor frames, viewed in RViz.
- [ ] **3. Base under ros2_control + teleop:** mecanum controller, DS4 driving, odometry checked against a tape measure.
- [ ] **4. Arm:** STS3215 bench scripts, then the C++ ros2_control hardware interface and MoveIt 2.
- [ ] **5. Sensors and perception:** ToF node, wrist camera, can detector and localizer.
- [ ] **6. Localization and navigation:** EKF, then Nav2.
- [ ] **7. Mission:** behavior tree that finds, picks up and delivers one can.
