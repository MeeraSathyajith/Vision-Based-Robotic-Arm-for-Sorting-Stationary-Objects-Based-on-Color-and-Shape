# Vision-Based-Robotic-Arm-for-Sorting-Stationary-Objects-Based-on-Color-and-Shape
Automated Color and Shape Based Sorting System using Dobot Magician in CoppeliaSim with Python and OpenCV.
# Automated Robotic Sorting System using Dobot Magician

## Overview
This project implements an automated color and shape-based object sorting system using the Dobot Magician robotic arm simulated in CoppeliaSim. The system integrates computer vision (OpenCV) with robotic motion control (Inverse Kinematics) to perform fully automated pick-and-place operations.

The robot detects objects on a conveyor belt, identifies their color and shape, and sorts them into designated bins.

---

## Features

- 4-DOF Serial Manipulator (Dobot Magician)
- Physics-based simulation in CoppeliaSim
- Python–Simulation communication using ZeroMQ Remote API
- Real-time image acquisition from vision sensor
- HSV-based color detection using OpenCV
- Shape detection using contour approximation
- Inverse kinematics for precise robot motion
- Automated conveyor control
- Fully continuous sorting operation

---

## Technologies Used

- Python
- OpenCV
- NumPy
- CoppeliaSim
- ZeroMQ Remote API
- Inverse Kinematics
- Denavit–Hartenberg Modeling

---

## System Architecture

1. Conveyor transports objects
2. Proximity sensor detects object
3. Conveyor stops
4. Vision sensor captures image
5. OpenCV processes image
6. Color and shape are classified
7. Inverse Kinematics computes joint angles
8. Robot picks object
9. Object placed in corresponding bin
10. System resets and repeats

---

## Robot Configuration

The Dobot Magician is modeled as a 4-DOF serial manipulator:

- Joint 1 – Base rotation (θ₁)
- Joint 2 – Shoulder (θ₂)
- Joint 3 – Elbow (θ₃)
- Joint 4 – Wrist rotation (θ₄)
- End Effector – Suction cup

---

## Simulation Setup

- Import Dobot Magician model in CoppeliaSim
- Add conveyor system
- Place colored geometric objects
- Add vision sensor above workspace
- Add proximity sensor at picking zone
- Configure physics and dynamics

---

## Image Processing Pipeline

- Capture image from vision sensor
- Convert RGB → BGR
- Convert BGR → HSV
- Apply color thresholding
- Perform noise filtering
- Detect contours
- Classify object shape
- Calculate centroid for picking

---

##  Sorting Logic

| Property       | Drop Location |
|---------------|--------------|
| Red Object    | Bin 1        |
| Blue Object   | Bin 2        |
| Triangle      | Bin 3        |

---

##  How to Run

1. Open CoppeliaSim scene
2. Start simulation
3. Run Python control script
4. Ensure ZeroMQ Remote API is enabled
5. Observe automated sorting

---

## Applications

- Industrial automation
- Smart manufacturing
- Vision-guided robotics
- Warehouse automation
- Robotics education

---

## Future Improvements

- Real hardware deployment
- Deep learning-based object detection
- Multi-object tracking
- Performance optimization
- 6-DOF manipulator upgrade

---

## 👨‍💻 Author
Your Name  
B.Tech / Robotics / Mechatronics  
