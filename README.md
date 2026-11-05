# Quadruped robot dog

An 8-servo robot dog that walks, turns, stands and sits, and takes moves from the position of a
face in a live camera stream.

Course project, Robotics (ME 604), IIT Bombay, Jan 2025 to Apr 2025, in a 6-member team.
Guide: Prof. Abhishek Gupta.

## The robot

Each of the four legs has an MG90S servo at the hip and one at the knee, so the robot has 8 servos
and 2 DOF per leg. The body chassis was designed in CAD and 3D-printed. An ESP8266 runs the robot
and takes move commands over Wi-Fi.

## What I did

- **Firmware.** I wrote the C++ firmware for the ESP8266 in the Arduino IDE. Walking and turning
  use servo keyframes that were tuned by hand.
- **Poses.** I added standing and sitting poses. Every leg bends through a greater angle when
  sitting than when standing.
- **Camera.** I streamed live video from an ESP32-S3 camera. Face detection with an OpenCV Haar
  cascade ran on a PC, by screen capture.
- **Face to move.** I mapped the face position to a move and sent it to the ESP8266 by HTTP GET.
  Face above the centre: walk forward. At the centre: stand. Below the centre: sit.
- **Cooldown.** I added a command cooldown in the vision loop, so the same move was never sent to
  the ESP8266 twice within 5 s.

## Hardware

| Part | Used |
|---|---|
| Controller | ESP8266 |
| Servos | 8 x MG90S, hip and knee on each leg |
| Body | chassis designed in CAD and 3D-printed, 150 x 97.5 mm |
| Legs | 67 mm femur and 75 mm tibia on each leg |
| Camera | ESP32-S3 camera, live video stream |
| Vision | OpenCV Haar-cascade face detection on a PC |

The original firmware is not in this repo.

## Related repos

Two later repos, written in 2026 after the course and separate from this robot's firmware:

- `leg-ik-gait`: inverse kinematics and a gait generator in C++ for a leg with two joints (hip and
  knee), like this robot's
- `robot-dog-ros2-sim`: a ROS 2 and Gazebo simulation of a robot dog with this robot's body and leg
  sizes
