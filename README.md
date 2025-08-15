# Multi-Vehicle AVP
*Distributed Automated Valet Parking Across Multiple Hosts*

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![Autoware](https://img.shields.io/badge/Autoware-2024.11-brightgreen?logo=autodesk)
![YOLOv5](https://img.shields.io/badge/YOLO-v5-red?logo=github)
![Zenoh](https://img.shields.io/badge/Zenoh-1.4.0-purple?logo=apache)
![AVP Node](https://img.shields.io/badge/AVP_Node-Orchestrator-black?logo=robotframework)
![Multi-Host](https://img.shields.io/badge/Topology-2_Host_Setup-yellow?logo=windows)
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
[https://zubxxr.github.io/multi-vehicle-avp](https://zubxxr.github.io/multi-vehicle-avp)

---

## License
This project is licensed under the Apache License 2.0.

---

## Contact
For questions, suggestions, or collaboration opportunities, feel free to reach out:

- **Author:** Zubair Islam  
- **Email:** zubxxr@gmail.com  
- **LinkedIn:** [linkedin.com/in/zubairislam02](https://www.linkedin.com/in/zubairislam02/)
