# Multi Vehicle Autonomous Valet Parking Using Two Machines

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
This step covers running the Zenoh bridge on both hosts with their respective config files. 

After Step 1, the current setup is:
- Autoware and AWSIM running on Host 1
- Autoware running on Host 2

AWSIM has been configured to simulate two ego vehicles, both publishing the same set of topics. However, the second vehicle, `AWSIM_EGO_Victus`, has `/vehicle1` manually prefixed to each of its topic names. This prevents conflicts between the two ego vehicles by isolating their data under separate namespaces.

**Example of Host 1 Ego Vehicle (AWSIM_EGO_ROG):**

![image](https://github.com/user-attachments/assets/0079be4c-058d-417a-b861-04158c2979e2)

**Example of Host 2 Ego Vehicle (AWSIM_EGO_ROG):**

![image](https://github.com/user-attachments/assets/2c768d2b-a6c0-4d36-aec5-00fbfc0a960d)


  
The Zenoh bridge is run on both hosts to enable communication and data exchange. On host 1, the [config](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/Zenoh-Setup/zenoh-bridge-awsim.json5) file has no namespace set, and the bridge is started first. This means it sends all ROS 2 topics to host 2 without any filtering. On host 2, the bridge is started shortly after, using a [config](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/Zenoh-Setup/zenoh-bridge-vehicle1.json5) that includes the namespace `/vehicle1`. As a result, it only receives data from the topics under that namespace. However, it can "see" other topics, but not recieve data from them. When these topics are listed on host 2, they appear without the `/vehicle1` prefix. 
This behavior works as expected and for convienience, as changing all the topics in Autoware would be challenging. Essentially, the Zenoh bridge on host 2 maps namespaced Zenoh topics to local ROS 2 topics by stripping the namespace defined in the config.

**Host 1 ROS2 Topics:**

![image](https://github.com/user-attachments/assets/010fc7f3-64c8-4104-b005-f408506e1d81)

**Host 2 ROS2 Topics:**

![image](https://github.com/user-attachments/assets/ad107ba4-30d5-429d-b8fb-3eef4007f90c)


### Host 1 (ROG Laptop)
**1. Run Zenoh Bridge**
   ``` bash
   cd $HOME/ZENOH/zenoh-plugin-ros2dds
   source $HOME/ZENOH/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c zenoh-bridge-awsim.json5
   ```

## Host 2 (Victus Laptop)
**1. Run Zenoh Bridge and Connect to Host 1**
   ``` bash
   cd $HOME/ZENOH/zenoh-plugin-ros2dds
   source $HOME/ZENOH/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c zenoh-bridge-vehicle1.json5 -e tcp/10.0.0.22:7447
   ```



### Launch YOLO Server
```cmd
cd $HOME/Multi-AVP
source env/bin/activate
python3 yolo_server.py
```



### Launch Script Sending Available Parking Spots to Autoware
```cmd
cd $HOME/Multi-AVP
source $HOME/autoware/install/setup.bash
source /opt/ros/humble/setup.bash
source env/bin/activate
python3 avp_sirc.py
```
