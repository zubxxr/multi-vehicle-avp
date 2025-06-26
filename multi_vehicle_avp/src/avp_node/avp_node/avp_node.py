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
    (-57.1, -28.6),
    (-55.2, -35.8),
    (-70.7, -31.9),
    (-68.3, -39.2)
]

drop_zone_polygon = Polygon(DROP_OFF_ZONE_POLYGON)

def is_in_drop_off_zone(x, y):
    return drop_zone_polygon.contains(Point(x, y))

drop_off_queue = []
drop_off_counter = 1
cars_in_zone = set()
car_id_to_label = {} 

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

    engage_auto_mode = "ros2 topic pub --once /autoware/engage autoware_vehicle_msgs/msg/Engage '{engage: True}' -1"
    
    # Closer one
    if args.debug:
        set_initial_pose = "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: 1749608320, nanosec: 716034807}, frame_id: 'map'}, pose: {pose: {position: {x: -71.57246398925781, y: -48.895652770996094, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7899923832424256, w: 0.6131166564520593}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'"

    head_to_drop_off = "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1749596637, nanosec: 370052949}, frame_id: 'map'}, pose: {position: {x: -57.330997467041016, y: -32.513362884521484, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.11762553592567591, w: 0.9930580211136696}}}' --once"

    parking_spot_locations = {
        1: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736234930, nanosec: 485255271}, frame_id: 'map'}, pose: {position: {x: -44.340084075927734, y: -14.542484283447266, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6048959869666175, w: 0.79630449261051}}}' --once",
        2: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736234964, nanosec: 268967027}, frame_id: 'map'}, pose: {position: {x: -41.737525939941406, y: -13.785665512084961, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6017180648737738, w: 0.7987085641237115}}}' --once",
        3: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235015, nanosec: 500375897}, frame_id: 'map'}, pose: {position: {x: -38.7567138671875, y: -12.567403793334961, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6019133113169981, w: 0.7985614351190562}}}' --once",
        4: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235477, nanosec: 950428247}, frame_id: 'map'}, pose: {position: {x: -35.9503173828125, y: -11.982339859008789, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6007336954096015, w: 0.799449202388447}}}' --once",
        5: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235517, nanosec: 532171546}, frame_id: 'map'}, pose: {position: {x: -32.98591232299805, y: -11.230877876281738, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.603871811065959, w: 0.7970814486612511}}}' --once",
        6: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235543, nanosec: 363285134}, frame_id: 'map'}, pose: {position: {x: -30.08383560180664, y: -10.155061721801758, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.608585037220489, w: 0.7934886593211878}}}' --once",
        7: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235564, nanosec: 777861615}, frame_id: 'map'}, pose: {position: {x: -27.20238494873047, y: -9.49226188659668, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6007326637076104, w: 0.7994499776438543}}}' --once",
        8: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235588, nanosec: 16728058}, frame_id: 'map'}, pose: {position: {x: -24.6075382232666, y: -8.993618965148926, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6036992773480252, w: 0.797212131449009}}}' --once",
        15: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235678, nanosec: 86261653}, frame_id: 'map'}, pose: {position: {x: -42.034812927246094, y: -22.750713348388672, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7969156099507567, w: 0.6040906476819629}}}' --once",
        16: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235705, nanosec: 118039164}, frame_id: 'map'}, pose: {position: {x: -39.109901428222656, y: -22.36135482788086, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7978757289989354, w: 0.6028219646582376}}}' --once",
        17: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235729, nanosec: 260929592}, frame_id: 'map'}, pose: {position: {x: -36.33027648925781, y: -21.73630142211914, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7999696817030807, w: 0.6000404222682599}}}' --once",
        18: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235765, nanosec: 496677516}, frame_id: 'map'}, pose: {position: {x: -33.3206672668457, y: -20.794353485107422, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7897854690370711, w: 0.6133831697217438}}}' --once",
        19: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235786, nanosec: 558316281}, frame_id: 'map'}, pose: {position: {x: -30.4317569732666, y: -19.839237213134766, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7983271798448894, w: 0.6022239732200185}}}' --once",
    }
    
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
            run_ros2_command(head_to_drop_off)
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
