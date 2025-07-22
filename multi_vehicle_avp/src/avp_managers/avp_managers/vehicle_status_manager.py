"""
Vehicle Status Manager Node

Tracks and relays vehicle status updates across multiple namespaces.

Responsibilities:
- Subscribes to status updates from all vehicles (e.g., "owner_left", "parking_engaged").
- Keeps latest status of each vehicle in memory.
- Allows other nodes to query or listen to current statuses (via a central topic).

Topics:
- /<namespace>/avp/status/update : Receives status string messages from vehicles.
- /<namespace>/avp/status/all    : Publishes all known vehicle statuses (String array or dict-style string).
- /avp/status/update and /avp/status/all are used for the default (global) channel.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import json
import ast

def get_topic(namespace, topic):
    """Returns topic string based on namespace."""
    return f"/{topic}" if namespace == "default" else f"/{namespace}/{topic}"

class VehicleStatusManager(Node):
    """Centralized status tracker for all vehicles."""
    def __init__(self):
        super().__init__('vehicle_status_manager')

        self.status_map = {}  # Format: {'vehicle1': 'owner_left', ...}
        self.status_publishers = {}

        # Declare and parse namespaces
        self.declare_parameter(
            'namespaces',
            "[]",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )

        try:
            raw_value = self.get_parameter('namespaces').value
            self.namespaces = ast.literal_eval(raw_value) if isinstance(raw_value, str) else raw_value
            
            # self.namespaces = self.get_parameter('namespaces').value
            if not all(isinstance(ns, str) for ns in self.namespaces):
                raise ValueError("Namespaces must be a list of strings.")
            
            # Always include global 'default' namespace
            if 'default' not in self.namespaces:
                self.namespaces.insert(0, 'default')
        except Exception as e:
            self.get_logger().error(f"Invalid namespaces: {e}")
            return

        self.get_logger().info(f"Managing vehicle status updates for: {self.namespaces}")

        # Set up topics per namespace
        for ns in self.namespaces:
            update_topic = get_topic(ns, "avp/status/update")
            all_vehicle_status_topic = get_topic(ns, "avp/status/all")

            self.create_subscription(String, update_topic, self.generate_callback(ns), 10)
            self.status_publishers[ns] = self.create_publisher(String, all_vehicle_status_topic, 10)

            self.get_logger().info(f"Listening for status updates on: {update_topic}")

        # Broadcast full status map every second
        self.timer = self.create_timer(1.0, self.broadcast_statuses)

    def generate_callback(self, ns):
        """Callback for status updates from vehicles."""
        def callback(msg):
            try:
                data = msg.data.strip()
                vehicle_id, status = data.split(":", 1)
                vehicle_id = vehicle_id.strip()
                status = status.strip()

                self.status_map[vehicle_id] = status
                self.get_logger().info(f"[{ns}] Status from {vehicle_id}: {status}")
            except Exception as e:
                self.get_logger().warn(f"Malformed status message: '{msg.data}'. Error: {e}")
        return callback

    def broadcast_statuses(self):
        """Publishes the current status map as a JSON string to all namespaces."""
        msg_out = String()
        msg_out.data = json.dumps(self.status_map)
        for ns, pub in self.status_publishers.items():
            pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleStatusManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
