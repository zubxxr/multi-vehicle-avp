# Multi-Vehicle AVP Simulation


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