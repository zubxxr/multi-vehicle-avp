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
