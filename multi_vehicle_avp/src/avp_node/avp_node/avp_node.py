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
--manual_localization true|false
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
    (-88.2, -53.4),
    (-89.7, -46.4),
    (-102.8, -57.4),
    (-105.0, -50.2)
]

drop_zone_polygon = Polygon(DROP_OFF_ZONE_POLYGON)

def is_in_drop_off_zone(x, y):
    return drop_zone_polygon.contains(Point(x, y))

drop_off_queue = []
drop_off_counter = 1
cars_in_zone = set()
car_id_to_label = {} 

parking_spot_goals = {
    1: {'x': -79.46, 'y': -33.53, 'z': 0.0, 'oz': -0.6063, 'ow': 0.7952},
    2: {'x': -76.62, 'y': -32.63, 'z': 0.0, 'oz': -0.6078, 'ow': 0.7941},
    3: {'x': -73.87, 'y': -31.64, 'z': 0.0, 'oz': -0.5938, 'ow': 0.8046},
    4: {'x': -70.97, 'y': -30.97, 'z': 0.0, 'oz': -0.5897, 'ow': 0.8077},
    5: {'x': -67.93, 'y': -30.15, 'z': 0.0, 'oz': -0.5952, 'ow': 0.8036},
    6: {'x': -65.31, 'y': -29.12, 'z': 0.0, 'oz': -0.5989, 'ow': 0.8008},
    7: {'x': -62.3, 'y': -28.18, 'z': 0.0, 'oz': -0.5955, 'ow': 0.8033},
    8: {'x': -59.79, 'y': -27.68, 'z': 0.0, 'oz': -0.6049, 'ow': 0.7963},
    9: {'x': -56.38, 'y': -26.39, 'z': 0.0, 'oz': -0.6087, 'ow': 0.7934},
    10: {'x': -52.88, 'y': -25.45, 'z': 0.0, 'oz': -0.5885, 'ow': 0.8085},
    11: {'x': -50.1, 'y': -24.57, 'z': 0.0, 'oz': -0.5998, 'ow': 0.8001},
    12: {'x': -47.29, 'y': -23.8, 'z': 0.0, 'oz': -0.5926, 'ow': 0.8055},
    13: {'x': -44.49, 'y': -22.98, 'z': 0.0, 'oz': -0.6151, 'ow': 0.7885},
    14: {'x': -77.22, 'y': -41.24, 'z': 0.0, 'oz': 0.8073, 'ow': 0.5902},
    15: {'x': -74.66, 'y': -40.47, 'z': 0.0, 'oz': 0.7944, 'ow': 0.6073},
    16: {'x': -71.69, 'y': -39.6, 'z': 0.0, 'oz': 0.8111, 'ow': 0.585},
    17: {'x': -68.77, 'y': -38.85, 'z': 0.0, 'oz': 0.7905, 'ow': 0.6124},
    18: {'x': -65.95, 'y': -37.8, 'z': 0.0, 'oz': 0.7942, 'ow': 0.6076},
    19: {'x': -62.87, 'y': -37.15, 'z': 0.0, 'oz': 0.804, 'ow': 0.5947},
    20: {'x': -60.0, 'y': -36.12, 'z': 0.0, 'oz': 0.8133, 'ow': 0.5818},
    21: {'x': -57.44, 'y': -35.6, 'z': 0.0, 'oz': 0.8008, 'ow': 0.5989},
    22: {'x': -54.26, 'y': -34.37, 'z': 0.0, 'oz': 0.7959, 'ow': 0.6054},
    23: {'x': -50.65, 'y': -33.35, 'z': 0.0, 'oz': 0.7811, 'ow': 0.6244},
    24: {'x': -47.74, 'y': -32.74, 'z': 0.0, 'oz': 0.7991, 'ow': 0.6012},
    25: {'x': -44.67, 'y': -32.27, 'z': 0.0, 'oz': 0.8076, 'ow': 0.5898},
    26: {'x': -42.46, 'y': -31.02, 'z': 0.0, 'oz': 0.7803, 'ow': 0.6254},
}

def generate_command(x, y, z, oz, ow):
    return f"""ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{{header: {{frame_id: "map"}}, pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0.0, y: 0.0, z: {oz}, w: {ow}}}}}}}' --once"""

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


    ## 'ros2 topic pub' is redundant. NEED TO CHANGE FOR ALL Publish commands
    engage_auto_mode = "ros2 topic pub --once /autoware/engage autoware_vehicle_msgs/msg/Engage '{engage: True}' -1"
    
    # Closer one
    if args.debug:
        set_initial_pose = "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: 1751258827, nanosec: 66529911}, frame_id: 'map'}, pose: {pose: {position: {x: -105.507080078125, y: -71.17322540283203, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7930478546598312, w: 0.6091593389413309}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'"

    head_to_drop_off = "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1751258650, nanosec: 3937974}, frame_id: 'map'}, pose: {position: {x: -90.51332092285156, y: -50.245758056640625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.13002967784495956, w: 0.9915101022579327}}}' --once"
    

    retrieve_vehicle_goal_pose = "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1751293184, nanosec: 319834439}, frame_id: 'map'}, pose: {position: {x: -97.19635009765625, y: -52.51005554199219, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9911820111935864, w: 0.13250743634316378}}}' --once"

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
                    print(f"[QUEUE] 🚫 Sent queue removal request for {queue_msg.data}")

                    if first_spot_in_queue not in avp_command_listener.reserved_spots_list:
                        msg = String()
                        msg.data = str(first_spot_in_queue)
                        avp_command_listener.reserved_spot_request_pub.publish(msg)
                        avp_command_listener.reserved_spots_list.append(first_spot_in_queue)
                        print(f"[AVP] ✅ Sent reservation request: {msg.data}")
                    else:
                        print(f"[AVP] ℹ️ Spot {first_spot_in_queue} already requested.")

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
            print(f"[AVP] 🗑 Sent removal request: {msg.data}")

            reserved_cleared = True

        if avp_command_listener.retrieve_vehicle and not retrieve_vehicle_complete:
            print("Going to Drop Off Zone.")
            run_ros2_command(retrieve_vehicle_goal_pose)
            run_ros2_command(engage_auto_mode)
            retrieve_vehicle_complete = True
           
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
