import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from utils.dropoff_and_parking import is_in_drop_off_zone
import time

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1
)

class AVPCommandListener(Node):
    def __init__(self, route_state_subscriber, motion_state_subscriber, args):
        super().__init__(f'avp_command_listener_{args.vehicle_id}')

        self.reserved_spots_list = []
        self.all_vehicle_status = {}
        self.current_queue = []
        self.cars_in_zone = set()

        self.ego_x = None
        self.ego_y = None
        self.vehicle_id = args.vehicle_id

        self.route_state_subscriber = route_state_subscriber
        self.motion_state_subscriber = motion_state_subscriber

        # States
        self.head_to_drop_off = False
        self.start_avp = False
        self.retrieve_vehicle = False
        self.status_initialized = False
        self.initiate_parking = False

        # Publishers
        self.vehicle_id_pub = self.create_publisher(String, "/avp/vehicle_id", qos_profile)
        self.local_status_publisher = self.create_publisher(String, '/avp/status', 10)
        self.all_status_publisher = self.create_publisher(String, '/avp/status/update', 10)
        self.queue_request_pub = self.create_publisher(String, 'avp/queue/request', 10)
        self.vehicle_count_request_pub = self.create_publisher(String, 'avp/vehicle_count/request', qos_profile)
        self.reserved_spot_request_pub = self.create_publisher(String, 'avp/reserved_parking_spots/request', 10)
        self.reserved_spot_remove_pub = self.create_publisher(String, 'avp/reserved_parking_spots/remove', 10)
        self.queue_remove_pub = self.create_publisher(String, 'avp/queue/remove', 10)

        # Subscribers   
        self.receive_button_command_sub = self.create_subscription(String, '/avp/command', self.command_callback, 10)
        self.all_vehicle_status_sub = self.create_subscription(String, '/avp/status/all', self.all_vehicle_status_callback, 10)
        self.is_in_drop_off_sub = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.queue_sub = self.create_subscription(String, '/avp/queue', self.queue_callback, 10)

    def all_vehicle_status_callback(self, msg):
        try:
            self.all_vehicle_status = eval(msg.data)
        except:
            self.get_logger().warn("Failed to parse /avp/status/all")

    def publish_vehicle_status(self, status: str):
        # Publish local status message
        self.local_status_publisher.publish(String(data=status))
        
        # Publish aggregated status with vehicle ID to manager
        tagged_status = f"{self.vehicle_id}:{status}"
        self.all_status_publisher.publish(String(data=tagged_status))

    def odom_callback(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y

        if not self.retrieve_vehicle and is_in_drop_off_zone(self.ego_x, self.ego_y): 
            if self.vehicle_id not in self.cars_in_zone:
                self.cars_in_zone.add(self.vehicle_id)
                msg = String()
                msg.data = self.vehicle_id
                self.queue_request_pub.publish(msg)
                print(f"[QUEUE REQUEST] Sent drop-off request for {self.vehicle_id}")
        else:
            if self.vehicle_id in self.cars_in_zone:
                self.cars_in_zone.remove(self.vehicle_id)                

    ## When a button in the AVPPanel is clicked, it sends different messages
    def command_callback(self, msg):
        if msg.data == "head_to_dropoff":
            self.head_to_drop_off = True
            self.publish_vehicle_status("Heading to drop-off zone")
        elif msg.data == "start_avp":
            self.start_avp = True
        elif msg.data == "retrieve":
            self.retrieve_vehicle = True

    def queue_callback(self, msg):
        try:
            self.current_queue = eval(msg.data)  # Make sure queue is sent as a list
        except Exception as e:
            self.get_logger().warn(f"Failed to parse /avp/queue: {e}")
            self.current_queue = []

    def handle_owner_exit(self, waiting_for_queue=False):
        self.publish_vehicle_status("Owner is exiting...")
        time.sleep(5)
        self.publish_vehicle_status("Owner has exited.")
        time.sleep(2)
        
        rclpy.spin_once(self.route_state_subscriber, timeout_sec=0.1)
        rclpy.spin_once(self.motion_state_subscriber, timeout_sec=0.1)

        if self.route_state_subscriber.state == 6:
            self.get_logger().info(f"[DEBUG] Reached state 6 after spin_once.")
            self.publish_vehicle_status("On standby...")
            return True
        
        # Keep retrying until route state == 6
        while self.route_state_subscriber.state != 6:
            rclpy.spin_once(self.route_state_subscriber, timeout_sec=0.2)

            if waiting_for_queue:
                return True
            else:
                if self.route_state_subscriber.state == 6:
                    self.publish_vehicle_status("On standby...")
                    return True
