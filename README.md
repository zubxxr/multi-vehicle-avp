# Multi Vehicle Autonomous Valet Parking

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

### UPDATED MAY 2025 - Kill the Unity Project Process When Stuck
- Just run this:
```cmd
kill -9 Unity
```  

### Kill the Unity Project Process When Stuck
![image](https://github.com/user-attachments/assets/a25e6660-b8f7-4b69-ace2-fd5720a115a0)

```cmd
ps aux | grep AWSIM-Labs
```
```cmd
kill -9 <PID>
```

```cmd
zubair@zubair-ROG-Zephyrus-G15:~/Multi-AVP$ ros2 topic echo /parking_spots/empty

data: '2025-01-07 01:45:22: 5,6,7,8,15,16,17,18,19,23,28'
---
data: '2025-01-07 01:45:24: 5,6,7,8,15,16,17,18,19,23,28'
---
data: '2025-01-07 01:45:26: 5,6,7,8,15,16,17,18,19,23,28'
---
data: '2025-01-07 01:45:28: 5,6,7,8,15,16,17,18,19,23,28'
```

### Launch Script Sending Available Parking Spots to Autoware
```cmd
cd $HOME/Multi-AVP
source $HOME/autoware/install/setup.bash
source /opt/ros/humble/setup.bash
source env/bin/activate
python3 yolo_server.py
```
