# DMV-AVP: Distributed Multi-Vehicle Autonomous Valet Parking
*Distributed Autonomous Valet Parking Across Multiple Hostss*

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

## Related Publication
This repository is part of the following paper:

**DMV-AVP: Distributed Multi-Vehicle Autonomous Valet Parking Using Autoware**  
Zubair Islam, Mohamed El-Darieby  
*Accepted at IEEE Intelligent Vehicles Symposium (IV) 2026, awaiting publication*  

Preprint: https://arxiv.org/abs/2601.16327

---

## Demo

[![DMV-AVP Demo](https://img.youtube.com/vi/o4xINcS6eKY/maxresdefault.jpg)](https://www.youtube.com/watch?v=o4xINcS6eKY)

## Features

- Multi-host, multi-vehicle AVP simulation
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
