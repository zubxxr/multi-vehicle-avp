#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class ReservedSpotPublisher(Node):
    def __init__(self):
        super().__init__('reserved_spot_publisher')
        self.publisher_ = self.create_publisher(String, '/parking_spots/reserved', 10)
        self.timer = self.create_timer(1.0, self.publish_once)

    def publish_once(self):
        msg = String()
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        msg.data = "Reserved Spots: [17]"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published reservation: "{msg.data}"')
        self.timer.cancel()  # publish once only

def main(args=None):
    rclpy.init(args=args)
    node = ReservedSpotPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()