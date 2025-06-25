import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.parameter import Parameter
from std_msgs.msg import String
from shapely.geometry import Point, Polygon
import sys

if '--help' in sys.argv or '-h' in sys.argv:
    print("""
🚗 Drop Off Zone Queue Manager Help

Listens for vehicle queue requests and broadcasts queue to all avp/queue topics.

✅ Example usage:
    ros2 run multi_avp drop_off_zone_queue_manager --ros-args -p namespaces:='["main", "vehicle2"]'

📌 Parameters:
    - namespaces: List of namespaces (e.g., ["main", "vehicle2"])
    """)
    sys.exit(0)

def get_topic(namespace, base_topic):
    return f"/{base_topic}" if namespace == "main" else f"/{namespace}/{base_topic}"

class QueueManager(Node):
    def __init__(self):
        super().__init__('queue_manager')
        
        self.valid = False
        self.queue = []
        self.queue_publishers = {}

        self.declare_parameter(
            'namespaces',
            ['main'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )

        try:
            self.namespaces = self.get_parameter('namespaces').value
            if not isinstance(self.namespaces, list) or not all(isinstance(ns, str) for ns in self.namespaces):
                raise ValueError("Invalid 'namespaces' list")
        except Exception as e:
            self.get_logger().error(f"❌ Invalid 'namespaces' parameter. Error: {e}")
            return

        for ns in self.namespaces:
            req_topic = get_topic(ns, "avp/queue/request")
            rem_topic = get_topic(ns, "avp/queue/remove")
            pub_topic = get_topic(ns, "avp/queue")

            self.create_subscription(String, req_topic, self.queue_request_callback, 10)
            self.create_subscription(String, rem_topic, self.queue_remove_callback, 10)
            self.queue_publishers[ns] = self.create_publisher(String, pub_topic, 10)

            self.get_logger().info(f"🛰️ Subscribed to: {req_topic}, {rem_topic}")
            self.get_logger().info(f"📤 Will publish to: {pub_topic}")

        self.timer = self.create_timer(1.0, self.publish_queues)
        self.valid = True
        self.get_logger().info(f"✅ Queue Manager Initialized for namespaces: {self.namespaces}")

    def queue_request_callback(self, msg):
        vehicle_id = msg.data.strip()
        if vehicle_id and vehicle_id not in self.queue:
            self.queue.append(vehicle_id)
            self.get_logger().info(f"➕ Added vehicle '{vehicle_id}' to queue.")
        elif vehicle_id:
            self.get_logger().info(f"ℹ️ Vehicle '{vehicle_id}' already in queue.")

    def queue_remove_callback(self, msg):
        vehicle_id = msg.data.strip()
        if vehicle_id in self.queue:
            self.queue.remove(vehicle_id)
            self.get_logger().info(f"➖ Removed vehicle '{vehicle_id}' from queue.")
        else:
            self.get_logger().info(f"🚫 Vehicle '{vehicle_id}' not found in queue.")

    def publish_queues(self):
        msg = String()
        msg.data = "[" + ", ".join(self.queue) + "]"
        for ns, pub in self.queue_publishers.items():
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QueueManager()

    if not getattr(node, 'valid', False):
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
