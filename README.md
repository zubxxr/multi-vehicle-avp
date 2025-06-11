# GURTTTTTTTTTTTTTTTTTTTTTTTT

## Build Command
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select 
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
