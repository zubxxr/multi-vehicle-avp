# Multi-Vehicle Autonomous Valet Parking System


![Autoware](https://img.shields.io/badge/Autoware-2024.11-blue?logo=autoware)
![AWSIM Labs](https://img.shields.io/badge/AWSIM%20Labs-Unity-green?logo=unity)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-purple?logo=ros)
![Zenoh](https://img.shields.io/badge/Zenoh-1.4.0-orange?logo=zenoh)
![YOLOv5](https://img.shields.io/badge/YOLO-v5-red?logo=github)
![AVP Node](https://img.shields.io/badge/AVP_Node-Python-black?logo=robotframework)
![License](https://img.shields.io/badge/License-Apache%202.0-blue?logo=apache)


A distributed, simulation-based system for **Automated Valet Parking (AVP)** using a multi-host setup.  
Integrates **Autoware**, **YOLOv5**, **Zenoh**, and the **Multi-Vehicle AVP Orchestration Module** for multi-vehicle parking coordination in AWSIM Labs.

---
  
## System Architecture
This section outlines the software stack, hardware specifications, and machine roles used throughout the project. The architecture is built around a distributed, multi-host setup where each host is responsible for specific tasks such as simulation, perception, control, or coordination.

### Software Stack and Version Overview

| **Component**              | **Name**                                | **Version / Branch**                               |
|---------------------------|-----------------------------------------|----------------------------------------------------|
| Operating System           | Ubuntu                                  | 22.04 LTS                                          |
| ROS 2 Distribution         | ROS 2                                   | Humble Hawksbill                                   |
| Autonomy Stack             | Autoware Universe                       | `release/2024.11` (forked and modified)            |
| Simulation Engine          | AWSIM Labs                              | Internal version (modified since Nov 2024)         |
| Middleware Bridge          | Zenoh Bridge for ROS 2                  | `release/1.4.0`                                    |
| Parking Spot Detection     | YOLO                                     | v5 (Ultralytics, custom-trained)                  |
| AVP Orchestration Module   | Multi-Vehicle AVP Node (Custom)         | Internal GitHub version (2025)                     |



### Roles of Each Machine

Each host machine is responsible for a specific set of modules in the distributed AVP system:

- **Host 1 (Nitro PC)** runs:
  - AWSIM Labs simulation (Unity-based)
  - YOLOv5-based parking spot detection server
  - Autoware (vehicle 1 stack)
  - AVP orchestration node (with **manager nodes** enabled)
  - Zenoh Bridge (in router mode)

- **Host 2 (ROG Laptop)** runs:
  - Autoware (vehicle 2 stack, with `/vehicle2` namespace)
  - A second AVP orchestration node (**no managers**, namespace-aware)
  - Zenoh Bridge (in client mode)

This setup is ideal for testing multi-vehicle behavior, decentralized planning, and V2X communication strategies in a scalable simulated environment.

---

## Software Installation and Setup
This section provides installation steps for each required component in the AVP system, including Autoware, Zenoh, Unity, and the YOLOv5 detection server.
> Refer to [Roles of Each Machine](roles-of-each-machine) to determine where each tool should be installed, depending on your setup.

### Repository Cloning
Before installing any softwares, start by cloning this repository:

```bash
cd ~
git clone https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking.git
```

This repository contains:
- The AVP orchestration node
- YOLOv5-based detection server
- Parking Spot Detection Unity code

---

### YOLOv5-Based Parking Spot Detection Module
The YOLOv5 server runs locally on the same machine as AWSIM Labs. It captures frames from the overhead camera in the simulation, performs vehicle detection using YOLOv5, extracts the bounding box coordinates, and sends them to Unity for further processing.

In Unity, custom scripts receive these bounding boxes, draws them on the overhead view, and compares them with predefined parking spot coordinates. If there is any overlap, the spot is marked as **occupied**.

As a result, a ROS 2 topic is published containing a list of currently **empty parking spots**, which can be visualized in the bottom-left corner of the simulation (see image below):

![image](https://github.com/user-attachments/assets/fd8fad9a-dfba-4936-b6e1-dfc06943eb2d)

#### Setup
Run the following commands to create the virtual environment and install requirements:
```bash
cd ~/Multi-Vehicle-Autonomous-Valet-Parking/yolo_detection_server 
python3 -m venv venv
source ~/Multi-Vehicle-Autonomous-Valet-Parking/yolo_detection_server/venv/bin/activate
pip install -r requirements.txt
```

### AVP Orchestration Node
The AVP Orchestration Node is the central control module that coordinates goal assignment, parking logic, and reservation handling across multiple vehicles. Each vehicle is assigned a dedicated AVP node instance—either with or without manager responsibilities—depending on the host’s role. The node also interfaces with RViz for visualization and command input.

This node must be built and installed on every host running an Autoware stack.
Its behavior depends on the host:

- Host 1 (with AWSIM Labs): runs the AVP node with manager responsibilities enabled. This instance oversees system-wide coordination, including assigning goals and managing parking spot reservations.
- Host 2 (or additional hosts): run namespace-aware AVP node instances without manager functionality, only controlling their respective vehicles.
    
#### Setup
To build the node, use the previously cloned repository:

```bash
cd ~/multi-vehicle-avp/multi_vehicle_avp/
colcon build
```

Be sure to source the workspace after building:
```bash
source install/setup.bash
```

---

## Launching the Full System: AWSIM Labs, Autoware, Zenoh, YOLOv5, and AVP Orchestration Node
Follow the steps below to launch all components required for the simulation.

### Step 1: Launching the Framework

[]

### Step 2: Start the YOLO Server
```bash
source ~/Multi-Vehicle-Autonomous-Valet-Parking/yolo_detection_server/venv/bin/activate
python3 yolo_server.py
```

If everything is working correctly, you will see output similar to the image below:
![image](https://github.com/user-attachments/assets/346d98c2-df20-48df-8cc1-311367c3021b)

  
### Step 3: Start the Automated Valet Parking Nodes

#### Host 1
```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/multi-vehicle-avp/multi_vehicle_avp/install/setup.bash
ros2 launch avp_node multi_avp_launch.py vehicle_id:=1 enable_managers:=true namespaces:="['vehicle2']"
```

#### Host 2
```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/multi-vehicle-avp/multi_vehicle_avp/install/setup.bash
ros2 launch avp_node multi_avp_launch.py vehicle_id:=2
```

**[Include Picture Here]**

---

## Future Work
[]


---


## Troubleshooting 
Refer to [Issues](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/issues) to see if your issue has been addressed. Otherwise, feel free to open one.

---

## Acknowledgements

Special thanks to [Ontario Tech University], [Prof. Mohamed El-Darieby], .... for their guidance and support.

## License

This project is licensed under the Apache License 2.0.

---
