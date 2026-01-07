# am-powder-cleanup-workflow

## 🔍 Overview
This repository provides a modular post-processing framework developed by the **Virginia Commonwealth University Additive Manufacturing (VCU AAM)** research group. 
It integrates computer vision, machine learning, and robotic path planning to automate powder cleanup of laser powder bed fusion additive manufacturing machines.

## Hardware
- Intel RealSense D435i
- Universal Robots UR5e

## Modules
| Module | Description |
|----------|--------------|
| **am_vision** | RGB and depth frame capture utilities. |
| **ml_vision** | Machine-learning interface layer that performs model inference (e.g., YOLOv5) to locate and distinguish build and feed cylinders. |
| **path_planner** | Grid-based motion planner that generates coverage paths around detected obstacles in a rastering pattern. |
| **robot** | Interface layer that executes robot motion commands, communicates with the robot, and collects robot position data. |

Each module can operate independently, but they are designed to form a cohesive vision-to-motion workflow.

## 📁 Repository Structure
```
am-powder-cleanup-workflow/
│── am_vision/         # Image alignment and preprocessing scripts (to be refactored)
│── ml_vision/         # ML-based object detection and mask generation
│── path_planner/      # Grid-based path planning and offset handling
│── robot_control/     # Robot execution and motion interface
│── contributing.md    # Contibution guidelines
│── master_script.py   # Example entry point
│── README.md          # This overview
```

## 🧩 Requirements
- Python 3.10.X
- `opencv`
- `numpy`
- `matplotlib`
- `ur_rtde`

## 📬 Contact
**VCU Additive Manufacturing Research Group**  
logan.schorr89@vcu.edu
