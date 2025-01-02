# AWSIM-Labs-SIRC

### To Build:
```cmd
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Launch AWSIM Labs
```cmd
cd awsim_labs_v1.5.2/ && ./awsim_labs.x86_64
```

### Launch E2E Autoware in Parking Map in AWSIM Labs
```cmd
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/ovin/autoware_map/parking_area/ launch_vehicle_interface:=true
```
