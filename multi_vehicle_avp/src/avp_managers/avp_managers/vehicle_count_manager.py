import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import sys

if '--help' in sys.argv or '-h' in sys.argv:
    print("""
🚗 Vehicle Count Manager Help

Listens for vehicle count requests and broadcasts count to all vehicle_count topics.

✅ Example usage:
    ros2 run multi_avp vehicle_count_manager --ros-args -p namespaces:='["main", "vehicle2"]'

📌 Parameters:
    - namespaces: List of namespaces (e.g., ["main", "vehicle2"])
    """)
    sys.exit(0)

def get_topic(namespace, topic):
    return f"/{topic}" if namespace == "main" else f"/{namespace}/{topic}"

class VehicleCountManager(Node):
    def __init__(self):
        super().__init__('vehicle_count_manager')
        self.valid = False

        self.declare_parameter(
            'namespaces',
            ['main'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )

        try:
            self.namespaces = self.get_parameter('namespaces').value
            if not isinstance(self.namespaces, list) or not all(isinstance(ns, str) for ns in self.namespaces):
                raise ValueError("Invalid type inside namespaces")
        except Exception as e:
            self.get_logger().error(f"❌ Invalid 'namespaces' parameter. Must be a list of strings. Error: {e}")
            return

        self.get_logger().info(f"✅ Managing vehicle_count for: {self.namespaces}")

        self.vehicle_count = 0
        self.count_publishers = {}

        for ns in self.namespaces:
            topic = get_topic(ns, "vehicle_count")
            self.count_publishers[ns] = self.create_publisher(Int32, topic, 10)

            request_topic = get_topic(ns, "vehicle_count_request")
            self.create_subscription(String, request_topic, self.generate_callback(ns), 10)
            self.get_logger().info(f"🛰️ Listening on: {request_topic}")

        self.timer = self.create_timer(1.0, self.publish_all_counts)
        self.valid = True

    def generate_callback(self, incoming_ns):
        def callback(msg):
            if msg.data == "add_me":
                self.vehicle_count += 1
                self.get_logger().info(f"[{incoming_ns}] 🚗 New vehicle joined. Total: {self.vehicle_count}")

                # Publish new count to all namespaces
                msg_out = Int32()
                msg_out.data = self.vehicle_count
                for ns, pub in self.count_publishers.items():
                    pub.publish(msg_out)
                    self.get_logger().info(f"🔁 Forwarded count {self.vehicle_count} to {get_topic(ns, 'vehicle_count')}")
        return callback

    def publish_all_counts(self):
        msg = Int32()
        msg.data = self.vehicle_count
        for ns, pub in self.count_publishers.items():
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleCountManager()

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
