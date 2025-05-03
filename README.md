
# Step 1

## Install UnityHub
https://terminalroot.com/how-to-install-unity-engine-on-ubuntu-via-appimage/

## Host 1 (My Laptop)
1. Open Unity Project and Run the Scene (Name: AWSIM-Labs-Zenoh)
     ```bash
     cd ~/Unity
     ./UnityHub.AppImage
     ```
2. Launch Autoware
     ```bash
     cd ~/autoware
     source /opt/ros/humble/setup.bash
     source install/setup.bash
     ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/zubair/autoware_map/sirc/ launch_vehicle_interface:=true
     ```


## Host 2 (Victus Laptop)
1. Launch Autoware
     ```bash
     cd ~/autoware
     source install/setup.bash
     ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/Zubair/autoware_map/sirc/ launch_vehicle_interface:=true
     ```
---

# Step 2
## Host 1 (My Laptop)
1. Run Zenoh Bridge
     ``` bash
     cd $HOME/ZENOH/zenoh-plugin-ros2dds
     source $HOME/ZENOH/zenoh-plugin-ros2dds/install/setup.bash
     zenoh_bridge_ros2dds -c zenoh-bridge-awsim.json5
     ```
Note that this bridge config file does not have a namespace.

## Host 2 (Victus Laptop)
1. Run Zenoh Bridge and Connect to Host 1
     ``` bash
     cd $HOME/ZENOH/zenoh-plugin-ros2dds
     source $HOME/ZENOH/zenoh-plugin-ros2dds/install/setup.bash
     zenoh_bridge_ros2dds -c zenoh-bridge-vehicle1.json5 -e tcp/10.0.0.22:7447
     ```
Note that this bridge config file has a namespace of `/vehicle1`. This is because the AWSIM config file sends ROS2 topics through Zenoh with the `/vehicle1` namespace.
