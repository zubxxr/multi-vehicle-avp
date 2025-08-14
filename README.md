# Multi-Vehicle Autonomous Valet Parking

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![Autoware](https://img.shields.io/badge/Autoware-2024.11-brightgreen?logo=autodesk)
![YOLOv5](https://img.shields.io/badge/YOLO-v5-red?logo=github)
![Zenoh](https://img.shields.io/badge/Zenoh-1.4.0-purple?logo=apache)
![AVP Node](https://img.shields.io/badge/AVP_Node-Orchestrator-black?logo=robotframework)
![Multi-Host](https://img.shields.io/badge/Topology-2_Host_Setup-yellow?logo=windows)
![License](https://img.shields.io/badge/License-Apache%202.0-blue?logo=apache)

A distributed, simulation-based system for **Automated Valet Parking (AVP)** using a multi-host setup.  
Integrates **Autoware**, **YOLOv5**, and **Zenoh** for multi-vehicle parking coordination in AWSIM Labs.

> **Note:** This repository contains only the AVP-specific workspace, nodes, and configs.  
> Full multi-host environment setup (AWSIM Labs, Zenoh, Autoware) is documented in the [Multi-Vehicle Simulation Framework](https://github.com/zubxxr/multi-vehicle-framework).

---

## Table of Contents
1. [Overview](#overview)  
2. [Repository Contents](#repository-contents)  
3. [Building the Workspace](#building-the-workspace)  
4. [Launching AVP](#launching-avp)  
5. [Demonstration Scenarios](#demonstration-scenarios)  
6. [Future Work](#future-work)  
7. [Troubleshooting](#troubleshooting)  
8. [Acknowledgements](#acknowledgements)  
9. [License](#license)  

---

## Overview
This AVP workspace provides:
- **AVP Orchestration Node**: Coordinates goal assignment, parking logic, and reservation handling across multiple vehicles.
- **YOLOv5 Parking Spot Detection**: Uses overhead camera feeds from AWSIM Labs to detect occupied/empty parking spots.
- **Namespace-Aware Multi-Vehicle Control**: Supports multiple Autoware instances via Zenoh topic bridging.

Each vehicle runs its own AVP node:
- **Host 1**: Vehicle 1 + manager node (assigns goals & manages reservations).
- **Host 2**: Vehicle 2 + namespace-aware AVP node (no manager logic).

---

## Repository Contents
multi-vehicle-avp/  
├── multi_vehicle_avp/ — AVP orchestration package  
│   ├── launch/ — Launch files for single & multi-vehicle AVP  
│   ├── src/ — Python source code for AVP nodes & managers  
│   └── config/ — Configuration files  
├── yolo_detection_server/ — YOLOv5-based parking spot detection server  
│   ├── requirements.txt  
│   └── yolo_server.py  
├── maps/ — Lanelet2 and point cloud maps  
├── zenoh_configs/ — Zenoh bridge configs for multi-host setup  
└── cyclonedds.xml — CycloneDDS config for ROS 2 middleware  

---

## Building the Workspace
1. Clone this repository:  
   `cd ~`  
   `git clone https://github.com/zubxxr/multi-vehicle-avp.git`  
   `cd multi-vehicle-avp`  

2. Build:  
   `colcon build`  

3. Source:  
   `source install/setup.bash`  

---

## Launching AVP
These instructions assume AWSIM Labs, Autoware, and Zenoh are already running.  
Refer to the [Multi-Vehicle Simulation Framework setup guide](https://github.com/zubxxr/multi-vehicle-framework) for bringing up the full environment.

**Host 1** (Vehicle 1, with managers):  
`source /opt/ros/humble/setup.bash`  
`source ~/autoware/install/setup.bash`  
`source ~/multi-vehicle-avp/install/setup.bash`  
`ros2 launch avp_node multi_avp_launch.py vehicle_id:=1 enable_managers:=true namespaces:="['vehicle2']"`

**Host 2** (Vehicle 2, namespace-aware):  
`source /opt/ros/humble/setup.bash`  
`source ~/autoware/install/setup.bash`  
`source ~/multi-vehicle-avp/install/setup.bash`  
`ros2 launch avp_node multi_avp_launch.py vehicle_id:=2`

---

## Demonstration Scenarios

### 1. Goal Navigation
Both vehicles localize and navigate to separate goals:
- In RViz, set `2D Pose Estimate` for each vehicle.
- Use the **Goal** tool to assign unique destinations.
- Observe both moving in AWSIM Labs without conflict.

### 2. Parking Maneuver
Assign goals in parking polygons:
- AVP manager assigns free spots from YOLOv5 detection.
- Vehicles navigate, park, and mark spots as occupied.

---

## Future Work
- Enhanced reservation management for high-density lots.  
- Cooperative maneuvers (merging, overtaking).  
- Infrastructure-assisted parking using shared camera feeds.  
- Additional concepts available in thesis paper (Section X.X).  

---

## Troubleshooting
- **Vehicle doesn’t move**: Check initial pose and goal position in RViz.  
- **Vehicle 2 stuck waiting for pose**: Confirm Zenoh bridge is connected and namespace is correct.  
- **No parking spots detected**: Verify YOLOv5 server is running and publishing.  

---

## Acknowledgements
Special thanks to **Ontario Tech University**, **Prof. Mohamed El-Darieby**, and collaborators for their guidance and support.

---

## License
Licensed under the [Apache 2.0 License](LICENSE).
