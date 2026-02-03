# Multi-Vehicle AVP Simulation

## Prerequisites
Ensure the following components are set up on each host:

- **Host 1**:

    - AWSIM Labs simulation (Unity-based)
    - U-YOLO module
    - Autoware (vehicle 1 stack)
    - The AVP Managers from the AVP-CF
    - Zenoh Bridge (in router mode)

- **Host 2**:

    - Autoware (vehicle 2 stack, with `/vehicle2` namespace)
    - The AVP Node from the AVP-CF
    - Zenoh Bridge (in client mode)

---

## Launch Sequence
Follow the steps below to launch all components required for the simulation.

1. **Launching the DMAVA**

    Follow the [Launch Sequence](https://zubxxr.github.io/distributed-multi-autonomous-vehicle-architecture/GettingStarted/Multi-VehicleSimulation/) in DMAVA.

2. **Start the YOLO Server**

    **Host 1**

    Go to Tab 1, split the terminal horizontally, and run the following commands in the bottom pane:

    ```bash
    source ~/multi-vehicle-avp/yolo_detection_server/venv/bin/activate
    python3 yolo_server.py
    ```
    
3. **Start the AVP Module**

    **Host 1**

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/autoware/install/setup.bash
    source ~/multi-vehicle-avp/multi_vehicle_avp/install/setup.bash
    ros2 launch avp_node multi_avp_launch.py vehicle_id:=1 enable_managers:=true namespaces:="['vehicle2']"
    ```

    **Host 2**

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/autoware/install/setup.bash
    source ~/multi-vehicle-avp/multi_vehicle_avp/install/setup.bash
    ros2 launch avp_node multi_avp_launch.py vehicle_id:=2
    ```

## Multi-Vehicle AVP Demonstration

<iframe width="560" height="315" src="https://www.youtube.com/embed/s1ufNOW7mmE?list=PL4MADLjXmDi1Q5XXCuFTEntWz1c_T50jd" title="YouTube video" frameborder="0" allowfullscreen></iframe>

---

**Next Steps:** To scale this setup beyond two vehicles, see [Scaling to More Vehicles](../../Scalability/ScalingToMoreVehicles/index.md). 