# Multi-Vehicle AVP
*Distributed Automated Valet Parking Across Multiple Hosts*

![Autoware](https://img.shields.io/badge/Autoware-2024.11-blue?logo=autoware)
![AWSIM Labs](https://img.shields.io/badge/AWSIM%20Labs-Unity-green?logo=unity)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-purple?logo=ros)
![Zenoh](https://img.shields.io/badge/Zenoh-1.4.0-orange?logo=zenoh)
![YOLOv5](https://img.shields.io/badge/YOLO-v5-red?logo=github)
![AVP Node](https://img.shields.io/badge/AVP_Node-Python-black?logo=robotframework)
![License](https://img.shields.io/badge/License-Apache%202.0-blue?logo=apache)

The **Multi-Vehicle AVP** system extends the **Multi-Vehicle Framework** to support **multiple vehicles operating in a shared simulation environment across different physical hosts**.  

This setup leverages **Zenoh** for ROS 2 topic synchronization, enabling each vehicle to operate its own Autoware stack while coordinating parking decisions in real-time.

---

## Features
- Multi-host, multi-vehicle AVP simulation
- Zenoh-based distributed ROS 2 topic bridging
- Parking spot reservation & conflict resolution
- Namespace-aware orchestration for individual vehicles
- Compatible with Ubuntu 22.04, ROS 2 Humble, and AWSIM Labs

---

## Getting Started and Documentation
See [Getting Started](GettingStarted/SystemArchitecture/index.md).

For a condensed list of frequently used commands, see the [Developer Quick Commands](DeveloperGuide/QuickCommands/index.md) page.