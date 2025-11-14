# Autonomous Waitress Robot Using LiDAR
### Technological University Mandalay  
**Department of Mechatronics Engineering**

**Supervised by:** Daw Su Myat Hlaing  
**Candidate:** Mg Kyi Lwin Han  

---

## 🔗 Table of Contents
- [Abstract](#abstract)
- [Introduction](#1-introduction)
- [Methodology](#2-methodology)
- [Overall Block Diagram](#3-overall-blockdiagram)
- [Overall Flowchart](#4-overall-flowchart)
- [Pin Connections](#5-pin-connections)
- [Test and Results](#6-test-and-results)
- [Conclusion](#7-conclusion)

---

## Abstract
This thesis aims to simulate and construct a ROS2-based autonomous waitress robot capable of delivering ordered items from a web-based menu to respective tables while avoiding static and dynamic obstacles.  
The system mainly uses **ROS 2**, **SLAM Toolbox**, **Nav2**, **URDF**, and **Gazebo** simulation.

The robot performs well in simulation and shows acceptable performance in real-world scenarios, although no image-processing method is used and wheel slipping occurs on uneven or sloped surfaces.

---

## 1. Introduction
The goal of this thesis is to address labor shortages in the restaurant industry by implementing a fully autonomous robotic waitress capable of navigation, delivery, and integration with a web-based ordering system.

### **Aims & Objectives**
- Design a differential-drive mobile robot platform using Raspberry Pi 5 and Arduino Uno.
- Implement SLAM using a LiDAR sensor for restaurant mapping.
- Develop autonomous navigation using the ROS 2 Nav2 stack.
- Simulate the system in Gazebo and RViz2.
- Create a QR-code-based web ordering system.
- Evaluate obstacle avoidance and path-following performance.

---

## 2. Methodology
This system uses SLAM to generate occupancy grid maps, combining wheel encoder data for odometry and 360-degree LiDAR measurements (RP-LIDAR A1) for drift correction.  
Based on the generated map:

- **Global & Local Costmaps** are created  
- **A\*** plans the shortest path  
- **DWA** generates real-time velocity commands  
- Real-time laser data handles dynamic obstacle avoidance.

### System Diagram
![System Diagram](img/system_diagram.png)

### Example Robot Task JSON
```json
GET /api/robot/task

{
  "task": "deliver",
  "table": 5,
  "orderId": 10,
  "pose": { "x": 2.3, "y": 1.1, "yaw": 1.57 }
}
