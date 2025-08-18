# Developer Quick Commands

This section provides frequently used commands for building, launching, and testing the framework. It is intended for developers who already have the environment set up and want a fast reference during iteration.

## Host 1

```bash
~/Unity/UnityHub.AppImage
```

```bash
cd ~/multi-vehicle-avp/yolo_detection_server/
source ~/multi-vehicle-avp/yolo_detection_server/venv/bin/activate
python3 yolo_server.py
```

```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=$HOME/autoware_map/sirc/ launch_vehicle_interface:=true
```

``` bash
source ~/zenoh-plugin-ros2dds/install/setup.bash
zenoh_bridge_ros2dds -c ~/multi-vehicle-avp/zenoh_configs/zenoh-bridge-awsim.json5
```

```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/multi-vehicle-avp/multi_vehicle_avp/install/setup.bash
ros2 launch avp_node multi_avp_launch.py vehicle_id:=1 enable_managers:=true namespaces:="['vehicle2']"
```

## Host 2

```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=$HOME/autoware_map/sirc/ launch_vehicle_interface:=true
```

``` bash
source ~/zenoh-plugin-ros2dds/install/setup.bash
zenoh_bridge_ros2dds -c ~/multi-vehicle-avp/zenoh_configs/zenoh-bridge-vehicle2.json5 -e tcp/10.0.0.172:7447
```
> Remember to replace IP.

```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/multi-vehicle-avp/multi_vehicle_avp/install/setup.bash
ros2 launch avp_node multi_avp_launch.py vehicle_id:=2
```

---

## Build Command

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <package_name>
```

---

## Autoware PSIM

```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=$HOME/autoware_map/sirc/ launch_vehicle_interface:=true
```
