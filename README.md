# Multi-Vehicle Autonomous Valet Parking
*Distributed Autonomous Valet Parking Across Multiple Hostss*

![Autoware](https://img.shields.io/badge/Autoware-2024.11-blue?logo=autoware)
![AWSIM Labs](https://img.shields.io/badge/AWSIM%20Labs-Unity-green?logo=unity)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-purple?logo=ros)
![Zenoh](https://img.shields.io/badge/Zenoh-1.4.0-orange?logo=zenoh)
![YOLOv5](https://img.shields.io/badge/YOLO-v5-red?logo=github)
![AVP Node](https://img.shields.io/badge/AVP_Node-custom-black?logo=robotframework)
![License](https://img.shields.io/badge/License-Apache%202.0-blue?logo=apache)

The **Multi-Vehicle Autonomous Valet Parking** system extends the [**Distributed Multi-Vehicle Architecture**](https://github.com/zubxxr/distributed-multi-vehicle-architecture) to support **coordinated autonomous parking across multiple vehicles and physical hosts** using **Autoware Universe**, **AWSIM Labs**, and **Zenoh**.  

It integrates a **Unity-based perception module (U-YOLO)** for parking-spot detection and a **Multi-Vehicle AVP Node** for orchestration, queuing, and reservation management. Together, these components enable **real-time distributed coordination**, **synchronized planning**, and **conflict-free multi-vehicle parking behavior**.

---

---

## Demo

[![Multi-Vehicle AVP Demo](https://img.youtube.com/vi/o4xINcS6eKY/maxresdefault.jpg)](https://www.youtube.com/watch?v=o4xINcS6eKY)

## Features

- Multi-host, multi-vehicle Automated Valet Parking (AVP) simulation
- Zenoh-based distributed ROS 2 topic bridging for synchronized operation
- Parking spot detection and reservation mechanisms
- Namespace-aware orchestration for individual Autoware stacks
- Custom RViz panel for streamlined AVP testing and monitoring
- Scalable to larger fleets and more complex simulation scenarios

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
