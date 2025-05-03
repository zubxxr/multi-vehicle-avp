# Multi Vehicle Autonomous Valet Parking

## Step 1: Running AWSIM and Autoware
This step covers running AWSIM and Autoware on Host 1, and another separate Autoware client on Host 2.

### Host 1 (ROG Laptop)
**1. Launch UnityHub**
  ```bash
  cd ~/Unity
  ./UnityHub.AppImage
  ```
**2. Launch AWSIM**

After launching UnityHub, open the project named `AWSIM-Labs-Zenoh` and click play to run the scene.

**3. Launch Autoware on Host 1**     
   ```bash
   cd ~/autoware
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/zubair/autoware_map/sirc/ launch_vehicle_interface:=true
   ```
     
### Host 2 (Victus Laptop)
1. Launch Autoware on Host 2  
     ```bash
     cd ~/autoware
     source install/setup.bash
     ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/Zubair/autoware_map/sirc/ launch_vehicle_interface:=true
     ```
---


## Step 2: Running Zenoh Bridge
This step covers running the Zenoh bridge on both hosts with their respective config files. AWSIM was configured to have a second ego vehicle, which has ROS2 topics with the namespace of `/vehicle1` manually added. Once Autoware is running on host 2, and the Zenoh bridge is connected between both hosts, host 2 only receives the ROS2 topics from AWSIM that 

### Host 1 (ROG Laptop)
**1. Run Zenoh Bridge**
   ``` bash
   cd $HOME/ZENOH/zenoh-plugin-ros2dds
   source $HOME/ZENOH/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c zenoh-bridge-awsim.json5
   ```
Note that this bridge config file does not have a namespace. 

## Host 2 (Victus Laptop)
**1. Run Zenoh Bridge and Connect to Host 1**
   ``` bash
   cd $HOME/ZENOH/zenoh-plugin-ros2dds
   source $HOME/ZENOH/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c zenoh-bridge-vehicle1.json5 -e tcp/10.0.0.22:7447
   ```
Note that this bridge config file has a namespace of `/vehicle1`. This is because the AWSIM config file sends ROS2 topics through Zenoh with the `/vehicle1` namespace.












### To Build:
```cmd
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Launch E2E Autoware in SIRC Map in AWSIM Labs
```cmd
cd $HOME/autoware
source install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/zubair/autoware_map/sirc/ launch_vehicle_interface:=true
```

### Launch YOLO Server
```cmd
cd $HOME/Multi-AVP
source env/bin/activate
python3 yolo_server.py
```

### Kill the Unity Project Process When Stuck
```cmd
kill -9 Unity
```  

### Launch Script Sending Available Parking Spots to Autoware
```cmd
cd $HOME/Multi-AVP
source $HOME/autoware/install/setup.bash
source /opt/ros/humble/setup.bash
source env/bin/activate
python3 avp_sirc.py
```
