# Autonomous Valet Parking (AVP) Node

This is the core controller node for a single autonomous valet parking (AVP) vehicle. It operates using a state machine architecture to coordinate each phase of the parking flow — from drop-off to parking to retrieval.

---

## Overview
This node manages the end-to-end flow for an AVP vehicle by subscribing to key topics, handling UI commands, and coordinating with a central manager through status, queue, and reservation messages. The system is designed to handle multiple vehicles simultaneously.

---

## State Machine Flow
```
INIT → WAIT_FOR_DROP_OFF → HANDLE_DROP_OFF → START_AVP →
PARK_VEHICLE → RESERVE_SPOT → CLEAR_RESERVATION →
RETRIEVE → FINALIZE_RETRIEVAL → COMPLETE
```

### State Descriptions
| State | Description |
|-------|-------------|
| `INIT` | Node startup, ROS 2 initialization, vehicle ID broadcast |
| `WAIT_FOR_DROP_OFF` | Awaiting user command to head to drop-off zone |
| `HANDLE_DROP_OFF` | Checks if vehicle is in queue, monitors front car status, handles owner exit |
| `START_AVP` | Begins the AVP flow and engages autonomous mode |
| `PARK_VEHICLE` | Detects available parking spot and navigates to it |
| `RESERVE_SPOT` | Sends reservation request for chosen spot |
| `CLEAR_RESERVATION` | Releases spot once car is successfully parked |
| `RETRIEVE` | Handles car recall for owner pickup |
| `FINALIZE_RETRIEVAL` | Simulates owner getting in and drives to new destination |
| `COMPLETE` | Final state; AVP process completed successfully |

---

## Topics Used

### Input Topics:
- `/avp/command`: UI commands ("head_to_dropoff", "start_avp", "retrieve")
- `/avp/status/all`: Receives status updates from all vehicles
- `/avp/queue`: Vehicle queue info
- `/avp/parking_spots`: List of currently unoccupied parking spots
- `/localization/kinematic_state`: Ego vehicle pose
- `/planning/mission_planning/route_selector/main/state`: Checks if vehicle reached destination
- `/api/motion/state`: Checks if vehicle is moving or stopped

### Output Topics:
- `/avp/status`: This vehicle’s local status (simple string)
- `/avp/status/update`: Aggregated status dictionary {vehicle_id: status}
- `/avp/queue/request` & `/avp/queue/remove`: Queue entry/removal requests
- `/avp/reserved_parking_spots/request` & `/remove`: Spot reservation or release
- `/avp/reserved_parking_spots`: Publishes updated spot reservations
- `/avp/vehicle_count/request`: Lets the manager know this vehicle has entered
- `/avp/vehicle_id`: Broadcasts this vehicle’s ID to the system
- `/planning/mission_planning/goal`: Publishes goal poses to Autoware
- `/autoware/engage`: Triggers vehicle movement

---

## Launch Usage
This node is meant to be launched using a ROS 2 launch file.

### Example:
```bash
ros2 launch avp_node multi_avp_launch.py \
  vehicle_id:=1 \
  enable_managers:=true \
  namespaces:='[main, vehicle2]' \
  debug:=true
```

### Launch Arguments:
| Argument | Description |
|----------|-------------|
| `vehicle_id` | Unique vehicle identifier (e.g., 1, 2) |
| `debug` | Enables simulated initial pose when true |
| `enable_managers` | Whether to launch manager nodes alongside |
| `namespaces` | List of namespaces for multi-vehicle coordination |

---

## File Structure
```
avp_node/
├── avp_node/
│   ├── avp_node.py                  # AVP Node
│   └── __init__.py
├── launch/
│   └── multi_avp_launch.py            # Launch file for full simulation
└── README.md                          # This file
```

---
