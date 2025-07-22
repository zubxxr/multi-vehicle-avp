"""
Drop-Off Zone Queue Manager Node

Maintains and publishes queue information for AVP vehicles requesting to access the drop-off zone.

Topics:
- /<namespace>/avp/queue/request : vehicle sends request to enter queue
- /<namespace>/avp/queue/remove  : vehicle exits the queue
- /<namespace>/avp/queue         : current global queue state
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String
import ast

def get_topic(namespace, base_topic):
    """Constructs full topic name based on namespace."""
    return f"/{base_topic}" if namespace == "default" else f"/{namespace}/{base_topic}"

class QueueManager(Node):
    """Node to manage a queue of vehicles attempting to access a drop-off zone."""
    def __init__(self):
        super().__init__('queue_manager')
        
        self.initialized = False  # Marks whether setup was successful
        self.queue = []  # List to store current queue
        self.queue_publishers = {}  # Maps namespace to its publisher
        
        # Declare 'namespaces' parameter (array of strings)
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
                raise ValueError("Invalid 'namespaces' list")
            
            # Always include global 'default' namespace
            if 'default' not in self.namespaces:
                self.namespaces.insert(0, 'default')
        except Exception as e:
            self.get_logger().error(f"Invalid 'namespaces' parameter. Error: {e}")
            return

        # Set up subscribers and publishers per namespace
        for ns in self.namespaces:
            request_topic = get_topic(ns, "avp/queue/request")
            remove_topic = get_topic(ns, "avp/queue/remove")
            queued_vehicles_topic = get_topic(ns, "avp/queue")

            self.create_subscription(String, request_topic, self.queue_request_callback, 10)
            self.create_subscription(String, remove_topic, self.queue_remove_callback, 10)
            self.queue_publishers[ns] = self.create_publisher(String, queued_vehicles_topic, 10)

            self.get_logger().info(f"Subscribed to: {request_topic}, {remove_topic}")
            self.get_logger().info(f"Will publish to: {queued_vehicles_topic}")

        # Timer to publish the current queue state periodically
        self.timer = self.create_timer(1.0, self.publish_queues)

        self.initialized = True
        self.get_logger().info(f"Queue Manager Initialized for namespaces: {self.namespaces}")

    def queue_request_callback(self, msg):
        """Handles a new vehicle queue request."""
        vehicle_id = msg.data.strip()
        if vehicle_id and vehicle_id not in self.queue:
            self.queue.append(vehicle_id)
            self.get_logger().info(f"Added vehicle '{vehicle_id}' to queue.")
        elif vehicle_id:
            self.get_logger().info(f"Vehicle '{vehicle_id}' already in queue.")

    def queue_remove_callback(self, msg):
        """Handles a request to remove a vehicle from the queue."""
        vehicle_id = msg.data.strip()
        if vehicle_id in self.queue:
            self.queue.remove(vehicle_id)
            self.get_logger().info(f"Removed vehicle '{vehicle_id}' from queue.")
        else:
            self.get_logger().info(f"Vehicle '{vehicle_id}' not found in queue.")

    def publish_queues(self):
        """Publishes the current queue state to all namespaces."""
        msg = String()
        msg.data = "[" + ", ".join(self.queue) + "]"
        for ns, pub in self.queue_publishers.items():
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QueueManager()

    # Only spin if initialization succeeded
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