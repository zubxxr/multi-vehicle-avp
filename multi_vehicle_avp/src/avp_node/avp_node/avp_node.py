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
    1: {'x': -84.05, 'y': -46.03, 'z': 0.0, 'oz': 0.8106, 'ow': 0.5856},
    2: {'x': -80.79, 'y': -44.78, 'z': 0.0, 'oz': 0.8084, 'ow': 0.5886},
    3: {'x': -77.42, 'y': -44.0, 'z': 0.0, 'oz': 0.8042, 'ow': 0.5944},
    4: {'x': -74.18, 'y': -42.91, 'z': 0.0, 'oz': 0.8037, 'ow': 0.595},
    5: {'x': -70.73, 'y': -42.01, 'z': 0.0, 'oz': 0.8084, 'ow': 0.5886},
    6: {'x': -67.39, 'y': -40.89, 'z': 0.0, 'oz': 0.807, 'ow': 0.5905},
    7: {'x': -64.22, 'y': -39.91, 'z': 0.0, 'oz': 0.8029, 'ow': 0.5962},
    8: {'x': -61.02, 'y': -39.02, 'z': 0.0, 'oz': 0.8058, 'ow': 0.5922},
    9: {'x': -57.6, 'y': -38.03, 'z': 0.0, 'oz': 0.8042, 'ow': 0.5944},
    10: {'x': -54.25, 'y': -37.16, 'z': 0.0, 'oz': 0.8017, 'ow': 0.5977},
    11: {'x': -50.96, 'y': -36.05, 'z': 0.0, 'oz': 0.8064, 'ow': 0.5914},
    12: {'x': -53.53, 'y': -28.45, 'z': 0.0, 'oz': -0.5986, 'ow': 0.801},
    13: {'x': -56.57, 'y': -29.46, 'z': 0.0, 'oz': -0.6074, 'ow': 0.7944},
    14: {'x': -59.87, 'y': -30.47, 'z': 0.0, 'oz': -0.5838, 'ow': 0.8119},
    15: {'x': -63.43, 'y': -31.28, 'z': 0.0, 'oz': -0.6079, 'ow': 0.794},
    16: {'x': -66.5, 'y': -32.43, 'z': 0.0, 'oz': -0.599, 'ow': 0.8007},
    17: {'x': -69.77, 'y': -33.03, 'z': 0.0, 'oz': -0.6083, 'ow': 0.7937},
    18: {'x': -73.23, 'y': -33.95, 'z': 0.0, 'oz': -0.6015, 'ow': 0.7989},
    19: {'x': -76.55, 'y': -35.16, 'z': 0.0, 'oz': -0.6092, 'ow': 0.793},
    20: {'x': -79.67, 'y': -35.86, 'z': 0.0, 'oz': -0.609, 'ow': 0.7932},
    21: {'x': -83.23, 'y': -36.86, 'z': 0.0, 'oz': -0.6009, 'ow': 0.7993},
    22: {'x': -86.37, 'y': -38.03, 'z': 0.0, 'oz': -0.6005, 'ow': 0.7996},
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

        self.vehicle_id = args.vehicle_id

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
        self.vehicle_id_pub = self.create_publisher(String, "/avp/vehicle_id", 10)
        self.status_publisher = self.create_publisher(String, '/avp/status', 10)
        self.status_update_publisher = self.create_publisher(String, '/avp/status/update', 10)
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
        

        self.status_all_sub = self.create_subscription(
            String,
            '/avp/status/all',
            self.status_all_callback,
            10
        )
        self.status_all_data = {}
                
        self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.ego_x = None
        self.ego_y = None

        self.queue_sub = self.create_subscription(
            String,
            '/avp/queue',
            self.queue_callback,
            10
        )
        self.current_queue = []  # stores the queue list

    def status_all_callback(self, msg):
        try:
            self.status_all_data = eval(msg.data)
        except:
            self.get_logger().warn("Failed to parse /avp/status/all")

    def publish_vehicle_status(self, status: str):
        msg = String()
        msg.data = f"{self.vehicle_id}:{status}"
        self.status_update_pub.publish(msg)

    def odom_callback(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        if is_in_drop_off_zone(self.ego_x, self.ego_y): 
            if self.vehicle_id not in cars_in_zone:
                cars_in_zone.add(self.vehicle_id)
                msg = String()
                msg.data = self.vehicle_id
                self.queue_request_pub.publish(msg)
                print(f"[QUEUE REQUEST] Sent drop-off request for {self.vehicle_id}")
        else:
            if self.vehicle_id in cars_in_zone:
                cars_in_zone.remove(self.vehicle_id)

    ## When a button in the AVPPanel is clicked, it sends different messages
    def command_callback(self, msg):
        if msg.data == "head_to_dropoff":
            self.head_to_drop_off = True
            self.status_publisher.publish(String(data="Heading to drop-off zone"))
            self.status_update_publisher.publish(String(data=f"{self.vehicle_id}:Heading to drop-off zone"))

        if msg.data == "start_avp":
            self.start_avp = True

        if msg.data == "retrieve":
            self.retrieve_vehicle = True

    def queue_callback(self, msg):
        try:
            # self.get_logger().info(f"Received queue data: {msg.data}")
            self.current_queue = eval(msg.data)  # Make sure queue is sent as a list
        except Exception as e:
            self.get_logger().warn(f"Failed to parse /avp/queue: {e}")
            self.current_queue = []

    def handle_owner_exit(self):
        self.status_publisher.publish(String(data="Owner is exiting..."))
        self.status_update_publisher.publish(
            String(data=f"{self.vehicle_id}:Owner is exiting...")
        )
        time.sleep(5)

        self.status_publisher.publish(String(data="Owner has exited."))
        self.status_update_publisher.publish(
            String(data=f"{self.vehicle_id}:Owner has exited...")
        )
        time.sleep(2)

        rclpy.spin_once(self.route_state_subscriber, timeout_sec=1)

        self.get_logger().info(f"[DEBUG] Route Status: {self.route_state_subscriber.state}")

        
        if self.route_state_subscriber.state == 6:
            self.status_publisher.publish(String(data="On standby..."))
            self.status_update_publisher.publish(
                String(data=f"{self.vehicle_id}:On standby...")
            )

        return True

        
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
        self.state = msg.state

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
        self.state = msg.state
    
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

    ## Timeout is needed to wait for the vehicle count request subscriber sent from the central manager node
    ## This is due to high traffic in Zenoh and subscribers/publishers coming in at random
    timeout = 15

    while avp_command_listener.vehicle_count_request_pub.get_subscription_count() == 0 and timeout > 0:
        avp_command_listener.get_logger().info("Waiting for /avp/vehicle_count/request subscriber...")
        time.sleep(1)
        timeout -= 1

    avp_command_listener.vehicle_id_pub.publish(String(data=str(avp_command_listener.vehicle_id)))

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


    leave_area_payload = (
        "{header: {frame_id: 'map'}, "
        "pose: {position: {x: -103.84678649902344, y: -106.02139282226562, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: -0.6071746465964653, w: 0.7945684039341468}}}"
    )
    leave_area_goal_pose = build_ros2_pub("/planning/mission_planning/goal", "geometry_msgs/msg/PoseStamped", leave_area_payload)

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
    left_for_new_destination = False

    
    while rclpy.ok():


        if not avp_command_listener.status_initialized:
            avp_command_listener.status_publisher.publish(String(data="Arrived at location."))
            avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Arrived at location."))
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
            avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Arrived at drop-off area."))


        motion_state_subscriber.get_logger().info(f"[DEBUG] Motion Status: {motion_state_subscriber.state}")

        if  (
            motion_state_subscriber.state == 1 and 
            not drop_off_completed and
            avp_command_listener.ego_x is not None and
            is_in_drop_off_zone(avp_command_listener.ego_x, avp_command_listener.ego_y)
        ):  
            
            
            if avp_command_listener.current_queue:

                if str(avp_command_listener.current_queue[0]) != str(avp_command_listener.vehicle_id):

                    avp_command_listener.get_logger().info(f"[DEBUG] Vehicle is NOT first in queue — waiting for vehicle ahead.")
                    
                    avp_command_listener.status_publisher.publish(String(data="Waiting 10 seconds for vehicle ahead..."))


                    first_in_line = avp_command_listener.current_queue[0]

                    last_known_status = avp_command_listener.status_all_data.get(str(first_in_line), "")

                    waited = 0
                    front_moved = False

                    while waited < 10:
                        loop_start = time.time()
                        rclpy.spin_once(avp_command_listener, timeout_sec=0.1)
                        new_status = avp_command_listener.status_all_data.get(str(first_in_line), "")

                        avp_command_listener.get_logger().info(f"Vehicle {first_in_line}'s status after {waited+1} sec: {new_status}")

                        if new_status != last_known_status:
                            avp_command_listener.get_logger().info(f"Status changed! {last_known_status} → {new_status}")
                            if ("Autonomous valet parking started" in new_status or "Waiting for an available parking spot..." in new_status):
                                front_moved = True
                                break
                                
                        last_known_status = new_status
                        waited += 1

                        time_to_wait = 1.0 - (time.time() - loop_start)
                        if time_to_wait > 0:
                            time.sleep(time_to_wait)

                    if front_moved:
                        avp_command_listener.status_publisher.publish(String(data="Vehicle ahead is preparing to leave. Proceeding..."))
                        avp_command_listener.status_update_publisher.publish(String(
                            data=f"{avp_command_listener.vehicle_id}:Vehicle ahead preparing to leave. Proceeding..."))
                        time.sleep(2)

                        ## Add code to let it move forward before doing the rest

                        ## Also need to display parking until it has arrived


                        avp_command_listener.handle_owner_exit()
                        drop_off_completed = True

                    else:
                        avp_command_listener.status_publisher.publish(String(data="Vehicle(s) ahead are still stationary. Proceeding with drop-off."))
                        avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Vehicle(s) ahead are still stationary. Proceeding with drop-off."))
                        time.sleep(2)
                        avp_command_listener.handle_owner_exit()
                        drop_off_completed = True

                    
                else:
                    avp_command_listener.get_logger().info(f"[DEBUG] Vehicle IS first in queue — proceeding with drop-off.")
                    avp_command_listener.handle_owner_exit()
                    drop_off_completed = True

            else:
                avp_command_listener.get_logger().warn(f"[WARN] Queue is empty!")

        if avp_command_listener.start_avp and not avp_command_listener.initiate_parking: 
            # start_avp_clicked = True
            avp_command_listener.status_publisher.publish(String(data="Autonomous valet parking started..."))
            avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Autonomous valet parking started..."))
            time.sleep(2)

            avp_command_listener.initiate_parking = True

            
        ############### START PARKING SPOT DETECTION ###############

        if avp_command_listener.initiate_parking and not parking_complete:

            rclpy.spin_once(parking_spot_subscriber, timeout_sec=0.1)  

            parking_spots = parking_spot_subscriber.available_parking_spots

            if parking_spots is None:
                avp_command_listener.status_publisher.publish(String(data="Waiting for an available parking spot..."))
                avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Waiting for an available parking spot..."))
                time.sleep(2)

            if parking_spots is not None:
                first_spot_in_queue = int(parking_spots.strip().strip('[]').split(',')[0].strip())

                if first_spot_in_queue != chosen_parking_spot:
                    if counter == 1:
                        print('\nParking spot #%s was taken.' % chosen_parking_spot)
                        counter = 0

                    # Printing the first value
                    print('\nAvailable parking spot found: %s' % first_spot_in_queue)

                    avp_command_listener.status_publisher.publish(String(data="Available parking spot found."))
                    avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Available parking spot found."))
                    time.sleep(2)
                    avp_command_listener.status_publisher.publish(String(data=f"Parking in Spot {first_spot_in_queue}."))
                    avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Parking in Spot {first_spot_in_queue}."))

                    parking_spot_goal_pose_command = parking_spot_locations[first_spot_in_queue]
                    run_ros2_command(parking_spot_goal_pose_command)
                    run_ros2_command(engage_auto_mode)

                    queue_msg = String()
                    queue_msg.data = avp_command_listener.vehicle_id
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
            avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Car has been parked."))

            msg = String()
            msg.data = str(chosen_parking_spot)
            avp_command_listener.reserved_spot_remove_pub.publish(msg)
            print(f"[AVP] Sent removal request: {msg.data}")

            reserved_cleared = True

            route_state_subscriber.state = -1

        if avp_command_listener.retrieve_vehicle and not retrieve_vehicle_complete:
            print("Going to Drop Off Zone.")
            avp_command_listener.status_publisher.publish(String(data="Car is called for retrieval."))
            avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Car is called for retrieval."))
            run_ros2_command(retrieve_vehicle_goal_pose)
            run_ros2_command(engage_auto_mode)
            retrieve_vehicle_complete = True
        
        if route_state_subscriber.state == 6 and retrieve_vehicle_complete and left_for_new_destination:
                avp_command_listener.status_publisher.publish(String(data="Car has been retrieved."))
                avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Car has been retrieved."))
                time.sleep(2)
                avp_command_listener.status_publisher.publish(String(data="Owner is getting in..."))
                avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Owner is getting in..."))
                time.sleep(2)
                avp_command_listener.status_publisher.publish(String(data="Owner is inside."))
                avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Owner is inside."))
                time.sleep(2)
                avp_command_listener.status_publisher.publish(String(data="Leaving to new destination."))
                avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Leaving to new destination."))
                run_ros2_command(leave_area_goal_pose)
                left_for_new_destination = True

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
