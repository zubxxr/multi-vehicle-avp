import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from utils.dropoff_and_parking import is_in_drop_off_zone
import time


cars_in_zone = set()

class AVPCommandListener(Node):
    def __init__(self, route_state_subscriber, motion_state_subscriber, args):
        super().__init__(f'avp_command_listener_{args.vehicle_id}')

        self.vehicle_id = args.vehicle_id

        self.reserved_spots_list = []
        
        self.route_state_subscriber = route_state_subscriber
        self.motion_state_subscriber = motion_state_subscriber

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

        rclpy.spin_once(self.route_state_subscriber, timeout_sec=0.1)
        rclpy.spin_once(self.motion_state_subscriber, timeout_sec=0.1)

        if self.route_state_subscriber.state == 6:
            self.get_logger().info(f"[DEBUG] Reached state 6 after spin_once.")
            self.status_publisher.publish(String(data="On standby..."))
            self.status_update_publisher.publish(
                String(data=f"{self.vehicle_id}:On standby...")
            )
            return True
        
        # Keep retrying until route state == 6
        while self.route_state_subscriber.state != 6:
            timeout = 5.0
            self.get_logger().info(f"[WAITING] Trying to confirm state=6...")

            while self.route_state_subscriber.state != 6 and timeout > 0:
                rclpy.spin_once(self.route_state_subscriber, timeout_sec=0.2)
                rclpy.spin_once(self.motion_state_subscriber, timeout_sec=0.2)
                timeout -= 0.2
                # self.get_logger().info(f"[WAIT] Route State = {self.route_state_subscriber.state}")

            if self.route_state_subscriber.state == 6:
                self.status_publisher.publish(String(data="On standby..."))
                self.status_update_publisher.publish(
                    String(data=f"{self.vehicle_id}:On standby...")
                )
                return True
            elif self.motion_state_subscriber.state == 3:
                self.status_publisher.publish(String(data="Moving up in queue..."))
                self.status_update_publisher.publish(
                    String(data=f"{self.vehicle_id}:Moving up in queue...")
                )
            else:
                self.status_publisher.publish(String(data="Waiting to proceed in queue..."))
                self.status_update_publisher.publish(
                    String(data=f"{self.vehicle_id}:Waiting to proceed in queue...")
                )
            