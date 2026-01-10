# Pavlov Mini – ROS2 Quadruped Robot 🤖

This repository contains the development of a **ROS2-based quadruped robot**
designed to autonomously detect and follow a red ball using onboard vision.

🚧 **Status:** Work in Progress

---

## 🚀 Features
- Quadruped locomotion using **CHAMP gait planner**
- ROS2 node-based control architecture
- Vision-based red ball detection (OpenCV + HSV)
- Modular software design for gait, control, and perception

---

## 🧠 System Architecture
- **High-level control:** Raspberry Pi (ROS2)
- **Low-level control:** Teensy microcontroller
- **Communication:** Serial / ROS2 topics
- **Vision:** USB camera

---

## 🛠 Technologies Used
- ROS2
- C++ / Python
- OpenCV
- CHAMP
- Raspberry Pi
- Teensy

---

## 📂 Workspace Structure
```text
pavlov_mini_ros2_ws/
├── src/
│   ├── pavlov_control
│   ├── pavlov_gait
│   ├── pavlov_description
│   └── pavlov_gazebo
# pavlov_mini_with_ros2
Quadruped, Autonomous robot capable of tracking objects using image processing.
