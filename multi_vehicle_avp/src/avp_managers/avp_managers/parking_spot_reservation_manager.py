"""
Parking Spot Reservation Manager Node

Tracks and manages reserved parking spots across multiple AVP vehicle namespaces.

Responsibilities:
- Subscribes to reservation and removal requests from each namespace.
- Maintains a global set of reserved spot IDs.
- Publishes the updated reservation list back to all participating namespaces.

Topics:
- /<namespace>/avp/reserved_parking_spots/request : Reserve a parking spot (expects String: spot_id)
- /<namespace>/avp/reserved_parking_spots/remove  : Release a reserved spot (expects String: spot_id)
- /<namespace>/avp/reserved_parking_spots         : Current list of all reserved spots
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String
import ast

# Helper function to format topics based on namespace
def get_topic(namespace, topic):
    return f"/{topic}" if namespace == "default" else f"/{namespace}/{topic}"

class ReservationManager(Node):
    def __init__(self):
        super().__init__('reservation_manager')

        self.initialized = False  # Will be set True only if initialization is successful
        self.reserved_spots = set()  # Global set of all reserved parking spots
        self.reserved_publishers = {}  # Publisher handles for each namespace

        # Declare and validate the 'namespaces' parameter
        self.declare_parameter(
            'namespaces',
            "[]",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )

        # Fetch and validate the parameter
        try:
            raw_value = self.get_parameter('namespaces').value
            self.namespaces = ast.literal_eval(raw_value) if isinstance(raw_value, str) else raw_value
            if not isinstance(self.namespaces, list) or not all(isinstance(ns, str) for ns in self.namespaces):
                raise ValueError("Invalid type inside namespaces")
            
            # Always include global 'default' namespace
            if 'default' not in self.namespaces:
                self.namespaces.insert(0, 'default')
        except Exception as e:
            self.get_logger().error(f"Invalid 'namespaces' parameter. Must be a list of strings. Error: {e}")
            return

        self.get_logger().info(f"Managing reservations for namespaces: {self.namespaces}")

        # Set up subscribers and publishers for each namespace
        for ns in self.namespaces:
            request_topic = get_topic(ns, "avp/reserved_parking_spots/request")
            remove_topic = get_topic(ns, "avp/reserved_parking_spots/remove")
            reserved_spots_topic = get_topic(ns, "avp/reserved_parking_spots")

            self.create_subscription(String, request_topic, self.generate_request_callback(ns), 10)
            self.create_subscription(String, remove_topic, self.generate_remove_callback(ns), 10)
            self.reserved_publishers[ns] = self.create_publisher(String, reserved_spots_topic, 10)

            self.get_logger().info(f"Listening on: {request_topic}, {remove_topic}")
            self.get_logger().info(f"Publishing to: {reserved_spots_topic}")

        self.initialized = True

    # Generate a callback for handling reservation requests
    def generate_request_callback(self, ns):
        def callback(msg):
            try:
                spot = int(msg.data.strip())
                if spot not in self.reserved_spots:
                    self.reserved_spots.add(spot)
                    self.get_logger().info(f"[{ns}] Reserved spot {spot}")
                    self.publish_all()
                else:
                    self.get_logger().info(f"[{ns}] Spot {spot} already reserved")
            except ValueError:
                self.get_logger().warn(f"[{ns}] Invalid spot request: '{msg.data}'")
        return callback
    
    # Generate a callback for handling reservation removals
    def generate_remove_callback(self, ns):
        def callback(msg):
            try:
                spot = int(msg.data.strip())
                if spot in self.reserved_spots:
                    self.reserved_spots.remove(spot)
                    self.get_logger().info(f"[{ns}] Removed spot {spot}")
                    self.publish_all()
                else:
                    self.get_logger().info(f"[{ns}] Spot {spot} was not in reservation list")
            except ValueError:
                self.get_logger().warn(f"[{ns}] Invalid spot removal: '{msg.data}'")
        return callback

    # Publishes the current list of reserved spots to all namespaces
    def publish_all(self):
        msg = String()
        msg.data = f"Reserved Spots: {sorted(list(self.reserved_spots))}"
        for ns, pub in self.reserved_publishers.items():
            pub.publish(msg)
            self.get_logger().info(f"Published to [{ns}]: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ReservationManager()

    if not getattr(node, 'initialized', False):
        node.destroy_node()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()