import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import String
import sys

if '--help' in sys.argv or '-h' in sys.argv:
    print("""
Parking Spot Reservation Manager Help

Listens for vehicle count requests and broadcasts count to all vehicle_count topics.

✅ Example usage:
    ros2 run multi_avp reservation_manager --ros-args -p namespaces:='["main", "vehicle2"]'

📌 Parameters:
    - namespaces: List of namespaces (e.g., ["main", "vehicle2"])
    """)
    sys.exit(0)

def get_topic(namespace, topic):
    return f"/{topic}" if namespace == "main" else f"/{namespace}/{topic}"

class ReservationManager(Node):
    def __init__(self):
        super().__init__('reservation_manager')

        self.valid = False
        self.reserved_spots = set()
        self.reserved_publishers = {}

        # Declare namespaces parameter
        self.declare_parameter(
            'namespaces',
            ['main'],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='List of namespaces to manage reservation topics for'
            )
        )

        try:
            self.namespaces = self.get_parameter('namespaces').value
            if not isinstance(self.namespaces, list) or not all(isinstance(ns, str) for ns in self.namespaces):
                raise ValueError("Invalid type inside namespaces")
        except Exception as e:
            self.get_logger().error(f"❌ Invalid 'namespaces' parameter. Must be a list of strings. Error: {e}")
            return

        self.get_logger().info(f"✅ Managing reservations for namespaces: {self.namespaces}")

        # Setup topics for each namespace
        for ns in self.namespaces:
            request_topic = get_topic(ns, "avp/reserved_parking_spots/request")
            remove_topic = get_topic(ns, "avp/reserved_parking_spots/remove")
            reserved_topic = get_topic(ns, "avp/reserved_parking_spots")

            self.create_subscription(String, request_topic, self.generate_request_callback(ns), 10)
            self.create_subscription(String, remove_topic, self.generate_remove_callback(ns), 10)
            self.reserved_publishers[ns] = self.create_publisher(String, reserved_topic, 10)

            self.get_logger().info(f"🛰️ Listening on: {request_topic}, {remove_topic}")
            self.get_logger().info(f"📡 Publishing to: {reserved_topic}")

        self.valid = True

    def generate_request_callback(self, ns):
        def callback(msg):
            try:
                spot = int(msg.data.strip())
                if spot not in self.reserved_spots:
                    self.reserved_spots.add(spot)
                    self.get_logger().info(f"[{ns}] ✅ Reserved spot {spot}")
                    self.publish_all()
                else:
                    self.get_logger().info(f"[{ns}] ℹ️ Spot {spot} already reserved")
            except ValueError:
                self.get_logger().warn(f"[{ns}] ⚠️ Invalid spot request: '{msg.data}'")
        return callback

    def generate_remove_callback(self, ns):
        def callback(msg):
            try:
                spot = int(msg.data.strip())
                if spot in self.reserved_spots:
                    self.reserved_spots.remove(spot)
                    self.get_logger().info(f"[{ns}] 🗑️ Removed spot {spot}")
                    self.publish_all()
                else:
                    self.get_logger().info(f"[{ns}] ℹ️ Spot {spot} was not in reservation list")
            except ValueError:
                self.get_logger().warn(f"[{ns}] ⚠️ Invalid spot removal: '{msg.data}'")
        return callback

    def publish_all(self):
        msg = String()
        msg.data = f"Reserved Spots: {sorted(list(self.reserved_spots))}"
        for ns, pub in self.reserved_publishers.items():
            pub.publish(msg)
            self.get_logger().info(f"🔁 Published to [{ns}]: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ReservationManager()

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
