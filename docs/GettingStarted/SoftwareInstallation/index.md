# Software Installation


This section provides installation steps for each required component in the AVP system, including Autoware, Zenoh, Unity, and the YOLOv5 detection server.

> Refer to [Host Roles](../SystemArchitecture/index.md/#host-roles) to determine where each tool should be installed.


### Repository Cloning
Before installing any softwares, start by cloning this repository:

```bash
cd ~
git clone https://github.com/zubxxr/multi-vehicle-avp.git
```

This repository contains:

- The AVP orchestration node
- YOLOv5-based detection server
- Parking Spot Detection Unity code

---

### YOLOv5-Based Parking Spot Detection Module
The YOLOv5 server runs locally on the same machine as AWSIM Labs. It captures frames from the overhead camera in the simulation, performs vehicle detection using YOLOv5, extracts the bounding box coordinates, and sends them to Unity to draw bounding boxes in the simulation.

As a result, a ROS 2 topic is published containing a list of currently **empty parking spots**, which can be visualized in the bottom-left corner of the simulation (see image below):

![image](https://github.com/user-attachments/assets/fd8fad9a-dfba-4936-b6e1-dfc06943eb2d)

#### Setup
Run the following commands to create the virtual environment and install requirements:
```bash
cd ~/multi-vehicle-avp/yolo_detection_server 
python3 -m venv venv
source ~/multi-vehicle-avp/yolo_detection_server/venv/bin/activate
pip install -r requirements.txt
```

### AVP Orchestration Node
The AVP Orchestration Node is the central control module that coordinates goal assignment, parking logic, and reservation handling across multiple vehicles. Each vehicle is assigned a dedicated AVP node instance—either with or without manager responsibilities—depending on the host’s role. The node also interfaces with RViz for visualization and command input.

This node must be built and installed on every host running an Autoware stack.
Its behavior depends on the host:

Host 1 (with AWSIM Labs)

- Runs the AVP node with manager responsibilities enabled. This instance oversees system-wide coordination, including assigning goals and managing parking spot reservations.

Host 2 (or additional hosts)

- Run namespace-aware AVP node instances without manager functionality, only controlling their respective vehicles.
    
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

**Next Steps:** Proceed to [Multi-Vehicle AVP Simulation](../Multi-VehicleAVPSimulation/index.md) to start the simulation.