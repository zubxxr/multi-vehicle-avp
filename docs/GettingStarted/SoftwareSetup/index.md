# Software Setup


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

### Autoware Universe


#### Version
This guide uses the Autoware branch [release/2024.11](https://github.com/autowarefoundation/autoware/tree/release/2024.11), with a forked and customized version available here: [Customized Autoware Repository](https://github.com/zubxxr/autoware/tree/2024.11-adapted)


USES DIFFERENT VIRSION

#### Installation Steps

The following installation steps are adapted from the [Autoware Universe Source Installation Guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

1. **Switch Branches** 


    ```bash
    git checkout avp-release/2025.08
    ```
TO DO

2. **Install Development Environment**

    ```bash
    ./setup-dev-env.sh
    ```
    
    > If any build issues are encountered, see the [Autoware Troubleshooting Guide](https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/#build-issues) and [this issue thread](https://github.com/zubxxr/multi-vehicle-framework/issues/24).

4. **Import Source Code**
    ```bash
    cd ~/autoware
    mkdir src
    vcs import src < autoware.repos
    ```

5. **Install ROS 2 Dependencies**
    ```bash
    source /opt/ros/humble/setup.bash
    sudo apt update && sudo apt upgrade
    rosdep update
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    ```

6. **Setup Ccache**
    ```bash
    sudo apt update && sudo apt install ccache
    mkdir -p ~/.cache/ccache
    touch ~/.cache/ccache/ccache.conf
    echo "max_size = 60G" >> ~/.cache/ccache/ccache.conf
    ```

7. **Integrate Ccache into Environment**

    Open the .bashrc file using the Text Editor:
    ```bash
    gedit ~/.bashrc
    ```
    Add the following lines:
    ```bash
    export CC="/usr/lib/ccache/gcc"
    export CXX="/usr/lib/ccache/g++"
    export CCACHE_DIR="$HOME/.cache/ccache/"
    ```
    Save the file and source it:
    ```bash
    source ~/.bashrc
    ```
    Verify:
    ```bash
    ccache -s
    ```

9. **Build Autoware**
    ```bash
    cd ~/autoware
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

    > Building Autoware can take about 1-3 hours.

    > If any build issues occur, refer to [issues](https://github.com/zubxxr/multi-vehicle-framework/issues) or the Autoware community for possible solutions.
    
---

#### Running Autoware

Once Autoware is successfully built, it should be tested to ensure the installation is functioning correctly.

1. Download the Map

    ```bash
    gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1prmoy4UBgT_J-tJ7SELsL7m9alWGzD71'
    unzip -d ~/autoware_map ~/autoware_map/sirc.zip
    ```

2. Source the Environment

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/autoware/install/setup.bash
    ```

3. Launch Autoware using the Planning Simulation

    ```bash
    ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sirc vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
    ```

    Follow the [Basic Simulations](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/#basic-simulations) to run different scenarios in Autoware.


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