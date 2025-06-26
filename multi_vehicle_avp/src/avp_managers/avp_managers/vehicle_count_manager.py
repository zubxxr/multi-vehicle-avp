"""
Vehicle Count Manager Node

Tracks and manages the total number of active AVP vehicles across multiple namespaces.

Responsibilities:
- Listens for vehicle count requests from each namespace.
- Maintains a global vehicle count.
- Publishes the updated count to all participating namespaces.

Topics:
- /<namespace>/avp/vehicle_count/request : Request to be added to the count (expects String: "add_me")
- /<namespace>/avp/vehicle_count         : Broadcasts the current global vehicle count (Int32)

To run manually:
ros2 run avp_managers vehicle_count_manager --ros-args -p namespaces:='["main", "vehicle2"]'
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import sys

if '--help' in sys.argv or '-h' in sys.argv:
    print("""
Vehicle Count Manager Help

Listens for vehicle count requests and broadcasts count to all vehicle_count topics.

Example usage:
    ros2 run multi_avp vehicle_count_manager --ros-args -p namespaces:='["main", "vehicle2"]'

Parameters:
    - namespaces: List of namespaces (e.g., ["main", "vehicle2"])
    """)
    sys.exit(0)

def get_topic(namespace, topic):
    """Returns the correct topic string based on namespace."""
    return f"/{topic}" if namespace == "main" else f"/{namespace}/{topic}"

class VehicleCountManager(Node):
    """Tracks the number of active AVP vehicles."""
    def __init__(self):
        super().__init__('vehicle_count_manager')

        self.initialized = False # Will be set True only if initialization is successful
        self.vehicle_count = 0 # Set initial count to 0
        self.count_publishers = {} # Publisher handles for each namespace


        # Declare and read the 'namespaces' parameter
        self.declare_parameter(
            'namespaces',
            ['main'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )

        # Fetch and validate the parameter
        try:
            self.namespaces = self.get_parameter('namespaces').value
            if not isinstance(self.namespaces, list) or not all(isinstance(ns, str) for ns in self.namespaces):
                raise ValueError("Invalid type inside namespaces")
        except Exception as e:
            self.get_logger().error(f"Invalid 'namespaces' parameter. Must be a list of strings. Error: {e}")
            return

        self.get_logger().info(f"Managing vehicle_count for: {self.namespaces}")

        # Setup publishers and subscribers per namespace
        for ns in self.namespaces:
            topic = get_topic(ns, "avp/vehicle_count")
            self.count_publishers[ns] = self.create_publisher(Int32, topic, 10)

            request_topic = get_topic(ns, "avp/vehicle_count/request")
            self.create_subscription(String, request_topic, self.generate_callback(ns), 10)
            self.get_logger().info(f"Listening on: {request_topic}")

        # Timer to re-broadcast current count every second
        self.timer = self.create_timer(1.0, self.publish_all_counts)
        self.initialized = True

    def generate_callback(self, incoming_ns):
        """Creates a callback function bound to a specific namespace."""
        def callback(msg):
            if msg.data == "add_me":
                self.vehicle_count += 1
                self.get_logger().info(f"[{incoming_ns}] New vehicle joined. Total: {self.vehicle_count}")

                # Publish new count to all namespaces
                msg_out = Int32()
                msg_out.data = self.vehicle_count
                for ns, pub in self.count_publishers.items():
                    pub.publish(msg_out)
                    self.get_logger().info(f"Forwarded count {self.vehicle_count} to {get_topic(ns, 'avp/vehicle_count')}")
        return callback

    def publish_all_counts(self):
        """Re-publishes the current vehicle count to all count topics."""
        msg = Int32()
        msg.data = self.vehicle_count
        for ns, pub in self.count_publishers.items():
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleCountManager()

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
