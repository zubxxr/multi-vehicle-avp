"""
Autonomous Valet Parking (AVP) Node

This is the core controller node that manages the behavior of a single AVP vehicle 
during a simulated autonomous valet parking flow. It handles commands from the user interface, 
monitors vehicle states, and publishes appropriate status updates and commands.

Responsibilities:
- Listens to AVP panel commands (e.g., head to drop-off, start AVP, retrieve).
- Checks if vehicle has entered the drop-off zone and manages queue requests.
- Waits for an available parking spot and navigates the vehicle to it.
- Coordinates reservation and release of parking spots across all vehicles.
- Publishes live status updates via /avp/status for user feedback.

Topics Used:
- /avp/command: String commands from the AVP panel.
- /localization/kinematic_state: Ego vehicle odometry.
- /parking_spots/empty: List of unoccupied spots, as a Stringified array.
- /avp/queue/request, /avp/queue/remove: Manage queue positions.
- /avp/vehicle_count/request: Notify the system about the presence of a new vehicle.
- /avp/reserved_parking_spots/request, /remove: Reserve or release a parking spot.
- /parking_spots/reserved: Broadcast currently reserved spots.
- /planning/mission_planning/goal: Topic to send goal poses to Autoware.
- /api/motion/state: Check if vehicle is stopped or moving.
- /planning/mission_planning/route_selector/main/state: Check if vehicle reached goal.
- /autoware/engage: Engages Autoware to start motion.

Usage (ROS 2 Launch File):
This script is designed to be launched from a ROS 2 launch file with arguments:
--vehicle_id <ID> 
--debug true|false
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.msg import MotionState
from tier4_planning_msgs.msg import RouteState
import argparse
from unique_identifier_msgs.msg import UUID
import uuid
from shapely.geometry import Point, Polygon
import subprocess
import time

DROP_OFF_ZONE_POLYGON = [
    (-94.9, -58.27),
    (-96.8, -50.6),
    (-113.4, -54.8),
    (-111.9, -62.8)
]

drop_zone_polygon = Polygon(DROP_OFF_ZONE_POLYGON)

def is_in_drop_off_zone(x, y):
    return drop_zone_polygon.contains(Point(x, y))

drop_off_queue = []
drop_off_counter = 1
cars_in_zone = set()
car_id_to_label = {} 

parking_spot_goals = {
    1: {'x': -86.97, 'y': -38.23, 'z': 0.0, 'oz': -0.6072, 'ow': 0.7946},
    2: {'x': -84.13, 'y': -37.28, 'z': 0.0, 'oz': -0.6072, 'ow': 0.7946},
    3: {'x': -81.19, 'y': -36.47, 'z': 0.0, 'oz': -0.6137, 'ow': 0.7896},
    4: {'x': -78.29, 'y': -35.51, 'z': 0.0, 'oz': -0.6131, 'ow': 0.79},
    5: {'x': -75.36, 'y': -34.89, 'z': 0.0, 'oz': -0.6072, 'ow': 0.7946},
    6: {'x': -72.82, 'y': -33.67, 'z': 0.0, 'oz': -0.5998, 'ow': 0.8002},
    7: {'x': -69.76, 'y': -32.9, 'z': 0.0, 'oz': -0.6279, 'ow': 0.7783},
    8: {'x': -67.22, 'y': -32.32, 'z': 0.0, 'oz': -0.5881, 'ow': 0.8088},
    9: {'x': -64.03, 'y': -31.16, 'z': 0.0, 'oz': -0.6011, 'ow': 0.7991},
    10: {'x': -60.19, 'y': -30.23, 'z': 0.0, 'oz': -0.5931, 'ow': 0.8051},
    11: {'x': -57.38, 'y': -29.58, 'z': 0.0, 'oz': -0.5921, 'ow': 0.8059},
    12: {'x': -54.57, 'y': -28.7, 'z': 0.0, 'oz': -0.575, 'ow': 0.8182},
    13: {'x': -51.96, 'y': -27.99, 'z': 0.0, 'oz': -0.5995, 'ow': 0.8004},
    14: {'x': -84.67, 'y': -46.67, 'z': 0.0, 'oz': 0.7946, 'ow': 0.6072},
    15: {'x': -82.02, 'y': -45.24, 'z': 0.0, 'oz': 0.8007, 'ow': 0.599},
    16: {'x': -79.2, 'y': -44.65, 'z': 0.0, 'oz': 0.8005, 'ow': 0.5994},
    17: {'x': -76.19, 'y': -43.66, 'z': 0.0, 'oz': 0.7792, 'ow': 0.6268},
    18: {'x': -73.32, 'y': -42.58, 'z': 0.0, 'oz': 0.8006, 'ow': 0.5992},
    19: {'x': -70.05, 'y': -41.93, 'z': 0.0, 'oz': 0.8013, 'ow': 0.5983},
    20: {'x': -67.3, 'y': -41.06, 'z': 0.0, 'oz': 0.8034, 'ow': 0.5954},
    21: {'x': -64.91, 'y': -40.18, 'z': 0.0, 'oz': 0.8111, 'ow': 0.5849},
    22: {'x': -61.67, 'y': -39.18, 'z': 0.0, 'oz': 0.7946, 'ow': 0.6072},
    23: {'x': -58.05, 'y': -38.08, 'z': 0.0, 'oz': 0.8173, 'ow': 0.5762},
    24: {'x': -55.08, 'y': -37.38, 'z': 0.0, 'oz': 0.8076, 'ow': 0.5898},
    25: {'x': -52.32, 'y': -36.34, 'z': 0.0, 'oz': 0.8131, 'ow': 0.5821},
    26: {'x': -49.84, 'y': -35.78, 'z': 0.0, 'oz': 0.784, 'ow': 0.6208},
}

def generate_command(x, y, z, oz, ow):
    return f"""ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{{header: {{frame_id: "map"}}, pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0.0, y: 0.0, z: {oz}, w: {ow}}}}}}}' --once"""


def build_ros2_pub(topic: str, msg_type: str, payload: str, once: bool = True) -> str:
    return f"ros2 topic pub {'--once ' if once else ''}{topic} {msg_type} '{payload}'"


parking_spot_locations = {
    spot_id: generate_command(**pose_dict)
    for spot_id, pose_dict in parking_spot_goals.items()
}

def generate_uuid():
    return UUID(uuid=list(uuid.uuid4().bytes))

class AVPCommandListener(Node):
    def __init__(self, route_state_subscriber, args):
        super().__init__(f'avp_command_listener_{args.vehicle_id}')

        self.reserved_spots_list = []
        
        self.route_state_subscriber = route_state_subscriber

        # States
        self.head_to_drop_off = False
        self.start_avp = False
        self.retrieve_vehicle = False
        self.status_initialized = False
        self.initiate_parking = False
        self.state = -1

        ## Publishers
        self.status_publisher = self.create_publisher(String, '/avp/status', 10)
        self.queue_request_pub = self.create_publisher(String, 'avp/queue/request', 10)
        self.vehicle_count_request_pub = self.create_publisher(String, 'avp/vehicle_count/request', 10)
        self.reserved_spot_request_pub = self.create_publisher(String, 'avp/reserved_parking_spots/request', 10)
        self.reserved_spot_remove_pub = self.create_publisher(String, 'avp/reserved_parking_spots/remove', 10)
        self.queue_remove_pub = self.create_publisher(String, 'avp/queue/remove', 10)

        self.reserved_timer = None
        self.current_reserved_spot = None

        self.subscription = self.create_subscription(
            String,
            '/avp/command',
            self.command_callback,
            10)
        
        self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.ego_x = None
        self.ego_y = None
        self.ego_id = f"car_{args.vehicle_id}"

    def odom_callback(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        if is_in_drop_off_zone(self.ego_x, self.ego_y): 
            if self.ego_id not in cars_in_zone:
                cars_in_zone.add(self.ego_id)
                msg = String()
                msg.data = self.ego_id
                self.queue_request_pub.publish(msg)
                print(f"[QUEUE REQUEST] Sent drop-off request for {self.ego_id}")
        else:
            if self.ego_id in cars_in_zone:
                cars_in_zone.remove(self.ego_id)

    ## When a button in the AVPPanel is clicked, it sends different messages
    def command_callback(self, msg):
        if msg.data == "head_to_dropoff":
            self.head_to_drop_off = True
            self.status_publisher.publish(String(data="Heading to drop-off zone"))

        if msg.data == "start_avp":
            self.start_avp = True

        if msg.data == "retrieve":
            self.retrieve_vehicle = True
            
class ParkingSpotSubscriber(Node):
    def __init__(self, args):
        super().__init__(f'parking_spot_subscriber_{args.vehicle_id}')
        self.available_parking_spots = None

        self.subscription = self.create_subscription(
            String,
            '/avp/parking_spots',
            self.available_parking_spots_callback,
            1)

    def available_parking_spots_callback(self, msg):
        self.available_parking_spots = msg.data

class RouteStateSubscriber(Node):
    def __init__(self, args):
        super().__init__(f'route_state_subscriber_{args.vehicle_id}')
        self.subscription = self.create_subscription(
            RouteState,
            '/planning/mission_planning/route_selector/main/state',
            self.route_state_callback,
            10)
        self.flag = False
        self.state = -1

    def route_state_callback(self, msg):
        # 6 - arrived
        if msg.state == 6:
            self.state = 6

class MotionStateSubscriber(Node):
    def __init__(self, args):
        super().__init__(f'motion_state_subscriber_{args.vehicle_id}')
        self.subscription = self.create_subscription(
            MotionState,
            '/api/motion/state',
            self.motion_state_callback,
            10)
        self.flag = False
        self.state = -1

    def motion_state_callback(self, msg):
        # 1 - stopped
        # 3 - moving
        if msg.state == 1:
            self.state = 1
    
def run_ros2_command(command):
    try:
        subprocess.run(command, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")

def main(args=None):
    
    parser = argparse.ArgumentParser(description="Run AVP with a specific vehicle ID.")
    parser.add_argument('--vehicle_id', type=str, required=True, help="Vehicle ID number only (e.g., 1 or 2)")
    parser.add_argument('--debug', type=str, default='false')
    args, unknown = parser.parse_known_args()
    args.debug = args.debug.lower() == 'true'

    rclpy.init(args=None)
    
    route_state_subscriber = RouteStateSubscriber(args)
    motion_state_subscriber = MotionStateSubscriber(args)
    avp_command_listener = AVPCommandListener(route_state_subscriber, args)
    parking_spot_subscriber = ParkingSpotSubscriber(args)
    reserved_spots_publisher = avp_command_listener.create_publisher(String, '/parking_spots/reserved', 10)

    rclpy.spin_once(avp_command_listener, timeout_sec=2.0)  # Give Zenoh time to sync

    # Wait for existing count from /vehicle_count topic
    rclpy.spin_once(avp_command_listener, timeout_sec=1.0)

    msg = String()
    msg.data = "add_me"
    avp_command_listener.vehicle_count_request_pub.publish(msg)
    print("[REQUEST] Sent vehicle_count_request to manager")

    # Send initial "N/A" reserved spot
    na_msg = String()
    na_msg.data = "[]"
    reserved_spots_publisher.publish(na_msg)

    # Send initial "N/A" drop-off queue
    dropoff_queue_msg = String()
    dropoff_queue_msg.data = "Drop-off Queue: []"

    engage_payload = "{engage: True}"
    engage_auto_mode = build_ros2_pub("/autoware/engage", "autoware_vehicle_msgs/msg/Engage", engage_payload)
    
    if args.debug:
        initial_pose_payload = (
            "{header: {frame_id: 'map'}, "
            "pose: {pose: {position: {x: -111.28318786621094, y: -80.02529907226562, z: 0.0}, "
            "orientation: {x: 0.0, y: 0.0, z: 0.7889132779275692, w: 0.614504548322938}}, "
            "covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, "
            "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
            "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}"
            "}"
        )
        set_initial_pose = build_ros2_pub("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", initial_pose_payload)

    drop_off_payload = (
        "{header: {frame_id: 'map'}, "
        "pose: {position: {x: -97.52228546142578, y: -55.04888916015625, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.11094320526941787, w: 0.9938267480826564}}}"
    )
    head_to_drop_off = build_ros2_pub("/planning/mission_planning/goal", "geometry_msgs/msg/PoseStamped", drop_off_payload)

    retrieve_payload = (
        "{header: {frame_id: 'map'}, "
        "pose: {position: {x: -105.69026947021484, y: -57.27252960205078, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: -0.9927605445395801, w: 0.12011037093222368}}}"
    )
    retrieve_vehicle_goal_pose = build_ros2_pub("/planning/mission_planning/goal", "geometry_msgs/msg/PoseStamped", retrieve_payload)

    # Simulate AWSIM initial pose
    if args.debug:
        run_ros2_command(set_initial_pose)

    counter = 0
    chosen_parking_spot = None
    drop_off_flag = True
    drop_off_completed = False
    parking_complete = False
    retrieve_vehicle_complete = False
    reserved_cleared = False
    
    while rclpy.ok():

        if not avp_command_listener.status_initialized:
            avp_command_listener.status_publisher.publish(String(data="Arrived at location."))
            avp_command_listener.status_initialized = True

        # If "Head to drop off" is clicked
        if drop_off_flag and avp_command_listener.head_to_drop_off:
            run_ros2_command(head_to_drop_off)
            run_ros2_command(engage_auto_mode)
            drop_off_flag = False

        if (    
            not avp_command_listener.initiate_parking and
            not drop_off_completed and
            avp_command_listener.ego_x is not None and
            is_in_drop_off_zone(avp_command_listener.ego_x, avp_command_listener.ego_y)
        ):
            # print("Drop-off valet destination reached.")
            avp_command_listener.status_publisher.publish(String(data="Arrived at drop-off area."))

        if  (
            motion_state_subscriber.state == 1 and 
            not drop_off_completed and
            avp_command_listener.ego_x is not None and
            is_in_drop_off_zone(avp_command_listener.ego_x, avp_command_listener.ego_y)
        ):
            avp_command_listener.status_publisher.publish(String(data="Owner is exiting..."))
            # time.sleep(7)
            avp_command_listener.status_publisher.publish(String(data="Owner has exited."))
            # time.sleep(2)
            drop_off_completed = True
            avp_command_listener.status_publisher.publish(String(data="On standby..."))

        if avp_command_listener.start_avp and not avp_command_listener.initiate_parking: 
            # start_avp_clicked = True
            avp_command_listener.status_publisher.publish(String(data="Autonomous valet parking started..."))
            avp_command_listener.initiate_parking = True
            # time.sleep(2)
            # avp_command_listener.status_publisher.publish(String(data="Waiting for an available parking spot..."))
            
        ############### START PARKING SPOT DETECTION ###############

        if avp_command_listener.initiate_parking and not parking_complete:

            rclpy.spin_once(parking_spot_subscriber, timeout_sec=0.1)  

            parking_spots = parking_spot_subscriber.available_parking_spots

            if parking_spots is None:
                avp_command_listener.status_publisher.publish(String(data="Waiting for an available parking spot..."))

            if parking_spots is not None:
                first_spot_in_queue = int(parking_spots.strip().strip('[]').split(',')[0].strip())

                if first_spot_in_queue != chosen_parking_spot:
                    if counter == 1:
                        print('\nParking spot #%s was taken.' % chosen_parking_spot)
                        counter = 0

                    # Printing the first value
                    print('\nAvailable parking spot found: %s' % first_spot_in_queue)

                    avp_command_listener.status_publisher.publish(String(data="Available parking spot found."))
                    time.sleep(2)
                    avp_command_listener.status_publisher.publish(String(data=f"Parking in Spot {first_spot_in_queue}."))

                    parking_spot_goal_pose_command = parking_spot_locations[first_spot_in_queue]
                    run_ros2_command(parking_spot_goal_pose_command)
                    run_ros2_command(engage_auto_mode)

                    queue_msg = String()
                    queue_msg.data = avp_command_listener.ego_id
                    avp_command_listener.queue_remove_pub.publish(queue_msg)
                    print(f"[QUEUE] Sent queue removal request for {queue_msg.data}")

                    if first_spot_in_queue not in avp_command_listener.reserved_spots_list:
                        msg = String()
                        msg.data = str(first_spot_in_queue)
                        avp_command_listener.reserved_spot_request_pub.publish(msg)
                        avp_command_listener.reserved_spots_list.append(first_spot_in_queue)
                        print(f"[AVP] Sent reservation request: {msg.data}")
                    else:
                        print(f"[AVP] Spot {first_spot_in_queue} already requested.")

                    route_state_subscriber.state = -1

                    print('Setting goal pose to parking spot:', first_spot_in_queue)

                    counter += 1
                    chosen_parking_spot = first_spot_in_queue

                    parking_complete = True
                
            time.sleep(1)
        ############### END PARKING SPOT DETECTION #################

        if route_state_subscriber.state == 6 and parking_complete and not reserved_cleared:
            avp_command_listener.status_publisher.publish(String(data="Car has been parked."))

            msg = String()
            msg.data = str(chosen_parking_spot)
            avp_command_listener.reserved_spot_remove_pub.publish(msg)
            print(f"[AVP] Sent removal request: {msg.data}")

            reserved_cleared = True

            route_state_subscriber.state = -1

        if avp_command_listener.retrieve_vehicle and not retrieve_vehicle_complete:
            print("Going to Drop Off Zone.")
            avp_command_listener.status_publisher.publish(String(data="Car is called for retrieval."))
            run_ros2_command(retrieve_vehicle_goal_pose)
            run_ros2_command(engage_auto_mode)
            retrieve_vehicle_complete = True
        

        if route_state_subscriber.state == 6 and retrieve_vehicle_complete:
                avp_command_listener.status_publisher.publish(String(data="Car has been retrieved."))

                ## Passenger getting in
                ## Leaving Area
                ## Gone

           
        rclpy.spin_once(route_state_subscriber, timeout_sec=1)
        rclpy.spin_once(motion_state_subscriber, timeout_sec=1)
        rclpy.spin_once(avp_command_listener, timeout_sec=1)

    route_state_subscriber.destroy_node()
    motion_state_subscriber.destroy_node()
    parking_spot_subscriber.destroy_node()
    avp_command_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
