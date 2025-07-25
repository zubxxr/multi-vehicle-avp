#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tier4_simulation_msgs.msg import DummyObject
from autoware_perception_msgs.msg import Shape, ObjectClassification
from geometry_msgs.msg import Pose, Quaternion, Vector3, Point
from unique_identifier_msgs.msg import UUID
from std_msgs.msg import Header, String
import uuid
import tf_transformations
import sys
import threading
import time
from datetime import datetime

def generate_uuid():
    return UUID(uuid=list(uuid.uuid4().bytes))

def make_quaternion(yaw_rad):
    q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

class ReservedSpotPublisher(Node):
    def __init__(self):
        super().__init__('reserved_spot_publisher')

        self.received_initial_list = False



        self.request_pub = self.create_publisher(String, '/parking_spots/reserved/request', 10)
        self.remove_pub = self.create_publisher(String, '/parking_spots/reserved/remove', 10)
        

        self.target_spot = 18
        
        self.timer = None

        self.queue_request_pub = self.create_publisher(String, '/queue_manager/request', 10)
        self.remove_queue_pub = self.create_publisher(String, '/queue_manager/remove', 10)


    def send_reservation_request(self, spot_id):
        msg = String()
        msg.data = str(spot_id)
        self.request_pub.publish(msg)
        self.get_logger().info(f"[NPC] Sent reservation request for spot: {msg.data}")

    def send_queue_request(self, car_id):
        msg = String()
        msg.data = car_id
        self.queue_request_pub.publish(msg)
        self.get_logger().info(f"[NPC] Sent queue request: {msg.data}")

    def send_queue_remove(self, car_id):
        msg = String()
        msg.data = car_id
        self.remove_queue_pub.publish(msg)
        self.get_logger().info(f"[NPC] Sent queue remove request: {msg.data}")



class NPCDummyCarPublisher(Node):

    npc_counter = 0
    
    def __init__(self, reserved_spot_publisher):
        super().__init__('npc_dummy_car_publisher')


        self.reserved_spot_publisher = reserved_spot_publisher



        self.pub = self.create_publisher(
            DummyObject,
            '/simulation/dummy_perception_publisher/object_info',
            10
        )

        

        self.active = True

        self.trajectories = [
            {"path": [(-58.1, -32.7, 0.25)], "max_v": 0.0, "min_v": 0.0, "delay": None},
            {"path": [(-58.1, -32.7, 0.25)], "max_v": 3.0, "min_v": 3.0, "delay": 9.0},
            {"path": [(-33.0, -26.4, 0.25)], "max_v": -1.0, "min_v": -1.0, "delay": 2.0},
            {"path": [(-34.6, -28.1, -5.25)], "max_v": 0.0, "min_v": 0.0, "delay": 2.0},
            {"path": [(-34.3, -26.9, -4.45)], "max_v": 1.0, "min_v": 1.0, "delay": 8.0},
            {"path": [(-36.4, -20.6, -4.45)], "max_v": 0.0, "min_v": 0.0, "delay": None},
        ]

        self.index = 0

        # Generate unique car ID
        NPCDummyCarPublisher.npc_counter += 1
        self.car_id = f"car_{NPCDummyCarPublisher.npc_counter}"

        print(self.reserved_spot_publisher.target_spot)

        self.reserved_spot_publisher.send_reservation_request(self.reserved_spot_publisher.target_spot)

        self.reserved_spot_publisher.send_queue_request(self.car_id)

        # Give the subscription some time to receive the current queue
        rclpy.spin_once(self, timeout_sec=0.5)

        self.object_id = generate_uuid()

        time.sleep(1.0)
        self.publish_static_car()
        threading.Thread(target=self.wait_for_input, daemon=True).start()


    def publish_static_car(self):
        traj = self.trajectories[0]
        self.publish_pose(traj["path"][0], traj["max_v"], traj["min_v"])
        self.get_logger().info("Published initial static car. Waiting for input...")

    def wait_for_input(self):
        self.get_logger().info("Waiting for input to start parking... (Press 'y' and Enter)")
        while True:
            user_input = sys.stdin.readline().strip().lower()
            if user_input == 'y':
                # self.get_logger().info("✅ Received input to start parking.")

                self.active = False


                # ✅ Remove from queue
                self.reserved_spot_publisher.send_queue_remove(self.car_id)
                self.run_trajectory_sequence()
                break

    def send_delete_all(self):
        obj = DummyObject()
        obj.header = Header()
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.header.frame_id = 'map'
        obj.action = DummyObject.DELETEALL
        self.pub.publish(obj)
        # self.get_logger().info("🗑️ Sent DELETE_ALL command.")

    def publish_pose(self, pos_tuple, max_v, min_v):
        x, y, yaw = pos_tuple

        obj = DummyObject()
        obj.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        obj.id = self.object_id

        pose = Pose()
        pose.position = Point(x=x, y=y, z=0.0)
        pose.orientation = make_quaternion(yaw)
        obj.initial_state.pose_covariance.pose = pose
        obj.initial_state.pose_covariance.covariance = [0.01] * 36
        obj.initial_state.twist_covariance.covariance = [0.0] * 36
        obj.initial_state.accel_covariance.covariance = [0.0] * 36

        classification = ObjectClassification()
        classification.label = ObjectClassification.CAR
        classification.probability = 1.0
        obj.classification = classification

        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        shape.dimensions = Vector3(x=4.0, y=1.8, z=2.0)
        obj.shape = shape

        obj.max_velocity = max_v
        obj.min_velocity = min_v
        obj.action = DummyObject.ADD

        self.pub.publish(obj)

    def run_trajectory_sequence(self):
        for i in range(1, len(self.trajectories)):  # skip first one (already static)
            traj = self.trajectories[i]

            self.send_delete_all()
            self.object_id = generate_uuid()
            self.publish_pose(traj["path"][0], traj["max_v"], traj["min_v"])

            delay = traj["delay"]
            if delay is not None:
                time.sleep(delay)

        removed = self.reserved_spot_publisher.target_spot



        # ✅ 1. Stop publishing first
        self.reserved_spot_publisher.stop_periodic_publishing()
        time.sleep(0.1)  # optional buffer to avoid race

        # ✅ 2. Then send remove request
        msg = String()
        msg.data = str(removed)
        self.reserved_spot_publisher.remove_pub.publish(msg)
        self.get_logger().info(f"🗑️ Requested to remove reserved spot: {msg.data}")        



                
def main(args=None):
    rclpy.init(args=args)

    reserved_spot_publisher = ReservedSpotPublisher()
    npc_dummy_car_publisher = NPCDummyCarPublisher(reserved_spot_publisher)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(npc_dummy_car_publisher)
    executor.add_node(reserved_spot_publisher)
    executor.spin()

    npc_dummy_car_publisher.destroy_node()
    reserved_spot_publisher.destroy_node()

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
