# Multi-Vehicle Autonomous Valet Parking System

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![Autoware](https://img.shields.io/badge/Autoware-2024.11-brightgreen?logo=autodesk)
![AWSIM Labs](https://img.shields.io/badge/AWSIM_Labs-2022.3.62f1-lightgrey?logo=unity)
![YOLOv5](https://img.shields.io/badge/YOLO-v5-red?logo=github)
![Zenoh](https://img.shields.io/badge/Zenoh-1.4.0-purple?logo=apache)
![AVP Node](https://img.shields.io/badge/AVP_Node-Orchestrator-black?logo=robotframework)
![Multi-Host](https://img.shields.io/badge/Topology-2_Host_Setup-yellow?logo=windows)
![License](https://img.shields.io/badge/License-MIT-blue?logo=open-source-initiative)

<p align="center">
  <img src="https://raw.githubusercontent.com/ros/ros2/rolling/ros2_logo.png" alt="ROS 2" height="40"/>
  <img src="https://raw.githubusercontent.com/autowarefoundation/autoware.ai/master/docs/media/autoware_logo.png" alt="Autoware" height="40"/>
  <img src="https://upload.wikimedia.org/wikipedia/commons/c/c4/Unity_2021.svg" alt="Unity" height="40"/>
  <img src="https://raw.githubusercontent.com/AWSIM-Labs/awsim_docs/main/static/img/logo_dark.svg" alt="AWSIM Labs" height="40"/>
  <img src="https://raw.githubusercontent.com/ultralytics/yolov5/master/docs/images/logo.png" alt="YOLOv5" height="40"/>
  <img src="https://zenoh.io/img/zenoh-logo.svg" alt="Zenoh" height="40"/>
  <img src="https://www.python.org/static/community_logos/python-logo.png" alt="Python" height="40"/>
</p>

A distributed, simulation-based system for autonomous valet parking using a distributed setup across two machines, integrating **AWSIM Labs** (Unity-based simulator), **Autoware** (AV Software Stack), **Zenoh** (communication middleware), and **YOLOv5** (object detection) to simulate and coordinate multiple autonomous vehicles navigating and parking in a shared environment.

This README is divided into three parts:  
1. A brief explanation of the system setup  
2. Instructions for installing and configuring all required software and components  
3. Steps to launch and run the full simulation system  
---

Topics are isolated using Zenoh namespaces to prevent conflicts, and custom map packages and detection pipelines are used to identify and manage available parking spots in real-time.

## Table of Contents

1. [System Setup](#system-setup)  
   - [Host 1](#host-1)  
   - [Host 2](#host-2)  
   - [Host 3](#host-3)  
2. [Software Installation and Setup](#software-installation-and-setup)  
   - [Autoware](#autoware)  
   - [AWSIM](#awsim)  
     - [Unity](#unity)  
     - [Open AWSIM Project](#open-awsim-project)  
     - [Replace the Map Package Link](#replace-the-map-package-link)
     - [Final Steps](#final-steps)
   - [Zenoh](#zenoh)
     - [How Zenoh Bridges Work](#how-zenoh-bridges-work)
     - [Installing Zenoh ROS 2 Bridge](#installing-zenoh-ros-2-bridge)
   - [YOLOv5](#yolov5)
     - [Download Required Files](#download-required-files)
     - [Setup Instructions](#setup-instructions)
   - [Optional: Barrier (Shared Keyboard and Mouse Across Hosts)](#optional-barrier-shared-keyboard-and-mouse-across-hosts)
3. [Launching the Full System](#launching-the-full-system-awsim-autoware-zenoh-and-yolo)  
   - [Step 1: Launching AWSIM](#step-1-launching-awsim)  
   - [Step 2: Start the YOLO Server](#step-2-start-the-yolo-server)  
   - [Step 3: Launching Autoware](#step-3-launching-autoware)  
   - [Step 4: Running Zenoh Bridge](#step-4-running-zenoh-bridge)  
   - [Step 5: Start the Automated Valet Parking Node](#step-5-start-the-automated-valet-parking-node)
  
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



### Hardware Specifications of Host Machines Used During Development

| **Host**        | **Model**                      | **CPU**                    | **GPU**                   | **RAM**  | **OS**          |
|----------------|--------------------------------|----------------------------|---------------------------|----------|-----------------|
| Nitro PC        | Acer Nitro N50-640             | Intel Core i7-12700F       | GeForce RTX 3060          | 24 GB    | Ubuntu 22.04    |
| ROG Laptop      | ASUS ROG Zephyrus G15 GA502IV  | AMD Ryzen 7 4800HS         | GeForce RTX 2060 Max-Q    | 24 GB    | Ubuntu 22.04    |

<img width="700" height="500" alt="image" src="https://github.com/user-attachments/assets/f329880a-004f-4e7c-bf47-3b05aeceb701" />


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
  - Zenoh Bridge (in router mode)

This setup is ideal for testing multi-vehicle behavior, decentralized planning, and V2X communication strategies in a scalable simulated environment.

---

## Software Installation and Setup

Before installing any softwares, start by cloning this repository:

```bash
cd ~
git clone https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking.git
```

### Barrier & AnyDesk (Shared Control and Remote File Access Across Hosts)
Managing multiple machines can be tedious, especially when frequently switching between keyboards, mice, or needing to transfer files across systems.

This setup originally used **AnyDesk** for remote control but eventually integrated **Barrier** for a more seamless, hardware-free workflow. While Barrier is ideal for controlling multiple machines with one keyboard and mouse, **AnyDesk remained essential for remote access and file transfer**, particularly after reboots.

#### Barrier: Shared Keyboard and Mouse Across Hosts

Barrier allows a single keyboard and mouse to control multiple systems by moving your cursor between screens as if they were part of one extended desktop. It greatly improves usability when running Autoware and other tools across multiple hosts.

#### AnyDesk: Remote Access and File Transfer

AnyDesk played a crucial supporting role throughout development:

- Remote Access: Host 1 (Nitro PC) operated without a dedicated keyboard or mouse. After reboots, the ROG laptop accessed it via AnyDesk to relaunch Barrier or terminal sessions.
- File Transfers: AnyDesk made it easy to share files, avoiding USB drives or external cloud services.
- Lightweight & Convenient: Its low overhead and persistent session features made it ideal for quick setup tasks and ongoing coordination.

#### Setup

1. **Initial Setup**:
   - Connect a keyboard and mouse to each system.
   - Install and configure **[AnyDesk](https://anydesk.com/en)** on all hosts.
     - Be sure to **set a password** in AnyDesk so you can connect automatically without needing to manually accept each time.
   - Install **[Barrier](https://github.com/debauchee/barrier)** on each host.
   - Set up Barrier:
     - On **Host 1** (the machine with your keyboard and mouse), set Barrier to run in **server** mode.
     - On **Host 2**, configure Barrier as a **client** and connect it to Host 1.

2. **Post-Reboot Recovery**:
   - Barrier may not auto-launch on reboot.
   - Instead of reconnecting physical peripherals, use **AnyDesk** to remotely access the host and launch Barrier.
   - Once Barrier is active, full keyboard/mouse control is restored.
  
> **Note**: While AnyDesk and Barrier are not part of the AVP runtime stack, they were vital for a smooth multi-host development experience. They helped reduce downtime, avoid errors, and speed up debugging during intensive multi-machine coordination.

### Autoware
Read the [following](https://autowarefoundation.github.io/autoware-documentation/main/installation/) to see the hardware requirements. 

The version of [Autoware](https://github.com/autowarefoundation/autoware/tree/release/2024.11) being used is `release/2024.11`. This version was forked and updated to better support the custom parking simulation use case.

#### Setup
To install Autoware, follow the instructions on [this page](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

> **Note:** Replace the following command:
> 
> ```bash
> git clone https://github.com/autowarefoundation/autoware.git
> ```
> 
> with:
> 
> ```bash
> git clone https://github.com/zubxxr/autoware.git
> ```

---

### AWSIM Labs

Setting up AWSIM Labs requires the installation of Unity. This whole section will reference the [AWSIM Labs Unity Setup](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/).

Follow the **"Environment preparation"** section and carefully read the **"ROS 2"** section to get started.

> The **"ROS 2"** section mentions that your environment should not have ROS 2 sourced.  
> It is recommended to **remove any ROS 2 sourcing lines from your `~/.bashrc`**, and instead **manually source ROS 2 only when needed**, to avoid environment conflicts.

#### Unity Installation

1. Install Unity Hub from the Package Manager
Start by installing Unity Hub to enable login. Follow the **"Install the Unity Hub on Linux"** section in [this page](https://docs.unity3d.com/hub/manual/InstallHub.html).

After installation, launch Unity Hub with:

```bash
unityhub
```
Sign in or create a Unity account as prompted.

2. Install Unity Editor Binary
Run the following commands to install the Unity Editor:
```bash
mkdir ~/Unity
cd ~/Unity
sudo apt install fuse libfuse2
sudo modprobe fuse
sudo groupadd fuse
sudo usermod -a -G fuse $USER
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage
chmod +x UnityHub.AppImage
./UnityHub.AppImage unityhub://2022.3.62f1/d91830b65d9b
```
The final command installs Unity version `2022.3.62f1`, which at the time of writing is the current version.

For future use, to launch Unity Hub later, run the following commands in a terminal that does not have ROS 2 sourced:
```bash
~/Unity/UnityHub.AppImage
```

These two steps complete the **"Unity installation**" section in the **AWSIM Labs Unity Setup**.

#### Open AWSIM Labs Project 
Follow the **"Open AWSIM Labs project**" step in the **AWSIM Labs Unity Setup**. 

> **Note:** At the time of writing, the documentation incorrectly tells you to clone:
> ```bash
> git clone git@github.com:autowarefoundation/AWSIM.git
> ```
> The correct repository is:
> ```bash
> git clone ~/https://github.com/autowarefoundation/AWSIM-Labs.git
> ```

#### Replace the Map Package Link
In the **"Import external packages"** section, **do not** use the green “Download Map Package” button shown in the docs.

Instead, **download the map package from this link**: [Download Zenoh-AWSIM-Labs-SIRC-June-4-2025.unitypackage](https://drive.google.com/file/d/1JXPlB_EWzItpGQwsTVuQIvlqlbNDCXrp/view?usp=sharing)

Then, follow the remaining steps to import the `.unitypackage` file into Unity.

#### Running the Demo
Lastly, follow the **Run the demo in Editor** section in the **AWSIM Labs Unity Setup** to run AWSIM Labs.

After successful completion, the simulation will be running (see image below), which will simulates two ego vehicles. These vehicles will later be controlled by their own Autoware clients. 

![image](https://github.com/user-attachments/assets/fffc4994-3622-4f69-a574-68b61ac352b1)

> The game view is expanded by double clicking on the **Game** tab.

---

### Zenoh
This step covers setting up the Zenoh bridge on both hosts using their respective config files. This enables communication between the two ego vehicles simulated in AWSIM Labs and their corresponding Autoware clients.

AWSIM Labs is configured to simulate two ego vehicles, both publishing identical sets of ROS 2 topics. To avoid topic collisions, the second vehicle is manually namespaced:

- `EgoVehicle_1` runs locally on the same host as AWSIM Labs and does **not** require a namespace (default `/`).
- `EgoVehicle_2` is bridged to a second machine running Autoware and uses the `/vehicle2` namespace.

This isolates their data and avoids topic collisions.

**Example of Host 1 Ego Vehicle (EgoVehicle_1):**

![image](https://github.com/user-attachments/assets/39639116-1c8b-48a2-88e1-d3f7cd109e05)

**Example of Host 2 Ego Vehicle (EgoVehicle_2):**

![image](https://github.com/user-attachments/assets/c5a7c99a-d0b0-42b4-86f2-ea0ef9b76d84)


#### Setup

1. Install [Rust](https://www.rust-lang.org/tools/install)

2. Clone and build the Zenoh bridge on all hosts:

```bash
cd ~
git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds -b release/1.4.0
cd ~/zenoh-plugin-ros2dds
rustup update
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select zenoh_bridge_ros2dds --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/zenoh-plugin-ros2dds/install/setup.bash
```

3. Find IP Address for Host 1
   
To set up Zenoh properly, you need the **IP address of the Host 1 active network interface** (usually Wi-Fi or Ethernet).

Run the following in your terminal:
```bash
ip a
```

Look for your active network interface:
- For **Wi-Fi**, the name usually starts with `wlp` (e.g., `wlp3s0`, `wlp0s20f3`)
- For **Ethernet**, it usually starts with `enp` (e.g., `enp2s0`)
- Ignore interfaces like `lo` (loopback), `docker0`, or `br-...` (Docker bridges)

Find the line that looks like this:
```
inet 10.0.0.172/24 ...
```

The IP address before the slash (`10.0.0.172`) is what you’ll use to connect to Host 1 from the other hosts:

```json
zenoh_bridge_ros2dds -e tcp/<IP-address>:7447
zenoh_bridge_ros2dds -e tcp/10.0.0.172:7447
```

> **Tip:** If you’re unsure which interface is active, check which one shows an `inet` address in the `10.x.x.x` or `192.168.x.x` range and says `state UP`.


3. **Launch the Zenoh bridge with host-specific configuration:**
   
   Both configs are available in the repository previously cloned. 
   ##### A) Host 1

   ```bash
   # This should be executed before Host 2
   source ~/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c ~/Multi-Vehicle-Autonomous-Valet-Parking/zenoh_configs/zenoh-bridge-awsim.json5
   ```
   
   ##### B) Host 2
   
   ```bash
   # This should be executed after Host 1 is running
   source ~/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c ~/Multi-Vehicle-Autonomous-Valet-Parking/zenoh_configs/zenoh-bridge-vehicle2.json5 -e tcp/10.0.0.172:7447
   ```

> **Note:** The IP address `10.0.0.172` used in the example above is from my local network setup.  
> You **must replace it with your own Host 1 IP address** obtained from the `ip a` command.

---

### YOLOv5
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
[]

#### Setup
```bash
cd ~/multi-vehicle-avp/multi_vehicle_avp/
colcon build
```

---

## Launching the Full System: AWSIM Labs, Autoware, Zenoh, YOLOv5, and AVP Orchestration Node
Follow the steps below to launch all components required for the simulation.

### Step 1: Launching AWSIM Labs
This step covers running AWSIM Labs on Host 1.

#### Host 1
**1. Launch UnityHub**
  ```bash
  ~/Unity/UnityHub.AppImage
  ```

**2. Launch AWSIM Labs**
See [Final Steps](final-steps).

### Step 2: Start the YOLO Server
```bash
source ~/Multi-Vehicle-Autonomous-Valet-Parking/yolo_detection_server/venv/bin/activate
python3 yolo_server.py
```

If everything is working correctly, you will see output similar to the image below:
![image](https://github.com/user-attachments/assets/346d98c2-df20-48df-8cc1-311367c3021b)

### Step 3: Launching Autoware
This step covers running Autoware on Host 1 and 2.

#### Host 1
```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=$HOME/autoware_map/sirc/ launch_vehicle_interface:=true
```
     
#### Host 2
```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=$HOME/autoware_map/sirc/ launch_vehicle_interface:=true
```
---

### Step 4: Running Zenoh Bridge
#### Host 1
```bash
source ~/zenoh-plugin-ros2dds/install/setup.bash
zenoh_bridge_ros2dds -c ~/Multi-Vehicle-Autonomous-Valet-Parking/zenoh_configs/zenoh-bridge-awsim.json5
```

#### Host 2
```bash
source ~/zenoh-plugin-ros2dds/install/setup.bash
zenoh_bridge_ros2dds -c ~/Multi-Vehicle-Autonomous-Valet-Parking/zenoh_configs/zenoh-bridge-vehicle2.json5 -e tcp/10.0.0.172:7447
```
> Use the IP address retrieved from [Installing Zenoh ROS 2 Bridge](#installing-zenoh-ros-2-bridge) step 3. In this case, its 10.0.0.172.
  
### Step 5: Start the Automated Valet Parking Nodes

#### Host 1
```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/Multi-AVP/multi_vehicle_avp/install/setup.bash
ros2 launch avp_node multi_avp_launch.py vehicle_id:=1 enable_managers:=true namespaces:="['vehicle2']"
```

#### Host 2
```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/Multi-AVP/multi_vehicle_avp/install/setup.bash
ros2 launch avp_node multi_avp_launch.py vehicle_id:=2
```

**[Include Picture Here]**


## Future Work
[]


---

## Troubleshooting 
Refer to [Issues](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/issues) to see if your issue has been addressed. Otherwise, feel free to open one.

---

## Acknowledgements

Special thanks to [Ontario Tech University], [Prof. Mohamed El-Darieby], and my lab collaborators for their guidance and support.

## License

[Specify license if needed]

---
