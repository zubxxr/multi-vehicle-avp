#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class ParkingSpotPublisher(Node):
    def __init__(self):
        super().__init__('parking_spot_publisher')
        self.publisher_ = self.create_publisher(String, '/avp/parking_spots', 10)
        self.subscription = self.create_subscription(
            String,
            '/avp/reserved_parking_spots',
            self.reserved_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.publish_parking_spots)

        # Initial state
        self.empty_spots = {4, 11, 16, 22}
        self.reserved_spots = set()

    def reserved_callback(self, msg):
        try:
            spot_str = msg.data.split(':')[-1].strip().strip('[]')
            new_spots = {int(s) for s in spot_str.split(',') if s.strip().isdigit()}
            self.reserved_spots.update(new_spots)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse reserved spots: {e}")

    def publish_parking_spots(self):
        available_spots = sorted(self.empty_spots - self.reserved_spots)
        msg = String()

        if available_spots:
            msg.data = f"[{', '.join(str(s) for s in available_spots)}]"
        else:
            msg.data = "Available Spots: []"

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSpotPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
