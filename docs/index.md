# DMV-AVP: Distributed Multi-Vehicle Autonomous Valet Parking
*Distributed Autonomous Valet Parking Across Multiple Hosts*

![Autoware](https://img.shields.io/badge/Autoware-2024.11-blue?logo=autoware)
![AWSIM Labs](https://img.shields.io/badge/AWSIM%20Labs-Unity-green?logo=unity)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-purple?logo=ros)
![Zenoh](https://img.shields.io/badge/Zenoh-1.7.2-orange?logo=zenoh)
![YOLOv5](https://img.shields.io/badge/YOLO-v5-red?logo=github)
![AVP Node](https://img.shields.io/badge/AVP_Node-custom-black?logo=robotframework)
![License](https://img.shields.io/badge/License-Apache%202.0-blue?logo=apache)

**DMV-AVP** extends [**DMAVA**](https://github.com/zubxxr/distributed-multi-vehicle-architecture) to support **coordinated AVP across multiple vehicles and physical hosts** using **Autoware Universe**, **AWSIM Labs**, and **Zenoh**.  

It adds the **U-YOLO** for parking-spot detection and **AVP-CF** modules for orchestration, queuing, and reservation management. Together, these components enable **real-time distributed coordination**, **synchronized planning**, and **conflict-free multi-vehicle parking behavior**.

---

## Features

- Multi-host, multi-vehicle AVP simulation
- Zenoh-based distributed ROS 2 topic bridging for synchronized operation
- Parking spot detection and reservation mechanisms
- Namespace-aware orchestration for individual Autoware stacks
- Custom RViz panel for streamlined AVP testing and monitoring
- Scalable to larger fleets and more complex simulation scenarios

---

## Getting Started

DMV-AVP builds on [DMAVA](https://github.com/zubxxr/distributed-multi-autonomous-vehicle-architecture). Make sure DMAVA is installed and set up before proceeding.  

Once DMAVA is ready, see [System Architecture](GettingStarted/SystemArchitecture/index.md).

## Developer Quick Commands

For a condensed list of frequently used commands, refer to [Developer Quick Commands](DeveloperGuide/QuickCommands/index.md).  


## Troubleshooting 

Refer to [DMAVA Issues](https://github.com/zubxxr/distributed-multi-autonomous-vehicle-architecture) to see if the issue has been addressed. Otherwise, feel free to open one.
