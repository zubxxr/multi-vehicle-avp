# GURTTTTTTTTTTTTTTTTTTTTTTTTTT

## Straight with lot

### Vehicle 1
Position: -81.34, -372.5, 267.31
Rotation: 0, 19.1, 0

### Vehicle 2
Position: -80.08, -372.5, 261.74
Rotation: 0, 19.1, 0


## Adjacent with lot

### Vehicle 1
Position: -259.37, -372.5, -198.02
Rotation: 0, -73.6, 0

### Vehicle 2
Position: -248.69, -372.5, -195.21
Rotation: 0, -73.6, 0


## Build Command
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select 
```
---

## Queue Planning Sim
```bash
ros2 launch autoware_launch planning_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/zubair/autoware_map/sirc/ launch_vehicle_interface:=true

```


---


## Host 1 (Nitro PC)
1. Open Unity Project and Run the Scene (Name: AWSIM-Labs-Zenoh)
     ```bash
     cd ~/Unity
     ./UnityHub.AppImage
     ```
     
## Host 2 (Victus Laptop)
1. Launch Autoware
     ```bash
     cd ~/autoware
     source install/setup.bash
     ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/Zubair/autoware_map/sirc/ launch_vehicle_interface:=true
     ```

## Host 3 (My Laptop)
1. Launch Autoware
     ```bash
     cd ~/autoware
     source /opt/ros/humble/setup.bash
     source install/setup.bash
     ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/zubair/autoware_map/sirc/ launch_vehicle_interface:=true
     ```
---

# Step 2
## Host 1 (Nitro PC)
1. Run Zenoh Bridge
   ``` bash
   cd $HOME/zenoh-plugin-ros2dds
   source $HOME/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c zenoh-bridge-awsim.json5
   ```
   
## Host 2 (Victus Laptop)
1. Run Zenoh Bridge and Connect to Host 1
   ``` bash
   cd $HOME/zenoh-plugin-ros2dds
   source $HOME/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c zenoh-bridge-vehicle1.json5 -e tcp/10.0.0.172:7447
   ```
   
## Host 3 (My Laptop)
1. Run Zenoh Bridge and Connect to Host 1
     ``` bash
     cd $HOME/zenoh-plugin-ros2dds
     source $HOME/zenoh-plugin-ros2dds/install/setup.bash
     zenoh_bridge_ros2dds -c zenoh-bridge-vehicle2.json5 -e tcp/10.0.0.172:7447
     ```
---
## Step 3: Start the Parking Spot Detection Node
### Launch YOLO Server
```cmd
cd $HOME/Multi-AVP
source env/bin/activate
python3 yolo_server.py
```

## Step 4: Start the Automated Valet Parking Node
```cmd
cd $HOME/Multi-AVP
source $HOME/autoware/install/setup.bash
source /opt/ros/humble/setup.bash
source env/bin/activate
python3 avp_sirc.py
```


## Step 5: Start the ACTUAL Automated Valet Parking Node
```cmd
cd $HOME/Multi-AVP/multi_avp_ws/
source $HOME/autoware/install/setup.bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch multi_avp_nodes multi_avp_launch.py avp_file:=avp_node vehicle_id:=1
```
