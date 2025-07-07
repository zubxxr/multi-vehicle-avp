"""
Autonomous Valet Parking (AVP) Node

This is the core controller node that manages the behavior of a single AVP vehicle 
during a simulated autonomous valet parking flow. It handles commands from the user interface, 
monitors vehicle states, and publishes appropriate status updates and commands.

Responsibilities:
- Listens to AVP panel commands (e.g., head to drop-off, start AVP, retrieve).
- Checks if vehicle has entered the drop-off zone and manages queue requests.
- Waits for an available parking spot and navigates the vehicle to it.
- Coordinates reservation and release of parking spots across all vehicles.
- Publishes live status updates via /avp/status for user feedback.

Topics Used:
- /avp/command: String commands from the AVP panel.
- /localization/kinematic_state: Ego vehicle odometry.
- /parking_spots/empty: List of unoccupied spots, as a Stringified array.
- /avp/queue/request, /avp/queue/remove: Manage queue positions.
- /avp/vehicle_count/request: Notify the system about the presence of a new vehicle.
- /avp/reserved_parking_spots/request, /remove: Reserve or release a parking spot.
- /parking_spots/reserved: Broadcast currently reserved spots.
- /planning/mission_planning/goal: Topic to send goal poses to Autoware.
- /api/motion/state: Check if vehicle is stopped or moving.
- /planning/mission_planning/route_selector/main/state: Check if vehicle reached goal.
- /autoware/engage: Engages Autoware to start motion.

Usage (ROS 2 Launch File):
This script is designed to be launched from a ROS 2 launch file with arguments:
--vehicle_id <ID> 
--debug true|false
"""

import rclpy
from std_msgs.msg import String
import argparse
import time

from vehicle.state_subscribers import RouteStateSubscriber, MotionStateSubscriber, ParkingSpotSubscriber
from vehicle.controller import AVPCommandListener
from utils.ros import run_ros2_command, build_ros2_pub
from utils.geometry import parking_spot_locations, is_in_drop_off_zone


def main(args=None):
    
    parser = argparse.ArgumentParser(description="Run AVP with a specific vehicle ID.")
    parser.add_argument('--vehicle_id', type=str, required=True, help="Vehicle ID number only (e.g., 1 or 2)")
    parser.add_argument('--debug', type=str, default='false')
    args, unknown = parser.parse_known_args()
    args.debug = args.debug.lower() == 'true'

    rclpy.init(args=None)
    
    route_state_subscriber = RouteStateSubscriber(args)
    motion_state_subscriber = MotionStateSubscriber(args)
    avp_command_listener = AVPCommandListener(route_state_subscriber, motion_state_subscriber, args)
    parking_spot_subscriber = ParkingSpotSubscriber(args)
    reserved_spots_publisher = avp_command_listener.create_publisher(String, '/parking_spots/reserved', 10)

    ## Timeout is needed to wait for the vehicle count request subscriber sent from the central manager node
    ## This is due to high traffic in Zenoh and subscribers/publishers coming in at random
    timeout = 15
    if args.debug: # if debug is true, that means planning simulator is used, meaning less traffic, so 5 is sufficient
        timeout = 5
    else: 
        timeout = 15

    while avp_command_listener.vehicle_count_request_pub.get_subscription_count() == 0 and timeout > 0:
        avp_command_listener.get_logger().info("Waiting for /avp/vehicle_count/request subscriber...")
        time.sleep(1)
        timeout -= 1

    avp_command_listener.vehicle_id_pub.publish(String(data=str(avp_command_listener.vehicle_id)))

    msg = String()
    msg.data = "add_me"
    avp_command_listener.vehicle_count_request_pub.publish(msg)
    print("[REQUEST] Sent vehicle_count_request to manager")

    # Send initial "N/A" reserved spot
    na_msg = String()
    na_msg.data = "[]"
    reserved_spots_publisher.publish(na_msg)

    # Send initial "N/A" drop-off queue
    dropoff_queue_msg = String()
    dropoff_queue_msg.data = "Drop-off Queue: []"

    engage_payload = "{engage: True}"
    engage_auto_mode = build_ros2_pub("/autoware/engage", "autoware_vehicle_msgs/msg/Engage", engage_payload)
    
    if args.debug:
        initial_pose_payload = (
            "{header: {frame_id: 'map'}, "
            "pose: {pose: {position: {x: -111.28318786621094, y: -80.02529907226562, z: 0.0}, "
            "orientation: {x: 0.0, y: 0.0, z: 0.7889132779275692, w: 0.614504548322938}}, "
            "covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, "
            "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
            "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}"
            "}"
        )
        set_initial_pose = build_ros2_pub("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", initial_pose_payload)

    drop_off_payload = (
        "{header: {frame_id: 'map'}, "
        "pose: {position: {x: -97.52228546142578, y: -55.04888916015625, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.11094320526941787, w: 0.9938267480826564}}}"
    )
    head_to_drop_off = build_ros2_pub("/planning/mission_planning/goal", "geometry_msgs/msg/PoseStamped", drop_off_payload)

    retrieve_payload = (
        "{header: {frame_id: 'map'}, "
        "pose: {position: {x: -105.69026947021484, y: -57.27252960205078, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: -0.9927605445395801, w: 0.12011037093222368}}}"
    )
    retrieve_vehicle_goal_pose = build_ros2_pub("/planning/mission_planning/goal", "geometry_msgs/msg/PoseStamped", retrieve_payload)


    leave_area_payload = (
        "{header: {frame_id: 'map'}, "
        "pose: {position: {x: -103.84678649902344, y: -106.02139282226562, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: -0.6071746465964653, w: 0.7945684039341468}}}"
    )
    leave_area_goal_pose = build_ros2_pub("/planning/mission_planning/goal", "geometry_msgs/msg/PoseStamped", leave_area_payload)

    # Simulate AWSIM initial pose
    if args.debug:
        run_ros2_command(set_initial_pose)

    counter = 0
    chosen_parking_spot = None
    drop_off_flag = True
    drop_off_completed = False
    parking_complete = False
    retrieve_vehicle_complete = False
    reserved_cleared = False
    left_for_new_destination = False

    
    while rclpy.ok():


        if not avp_command_listener.status_initialized:
            avp_command_listener.status_publisher.publish(String(data="Arrived at location."))
            avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Arrived at location."))
            avp_command_listener.status_initialized = True

        # If "Head to drop off" is clicked
        if drop_off_flag and avp_command_listener.head_to_drop_off:
            run_ros2_command(head_to_drop_off)
            run_ros2_command(engage_auto_mode)
            drop_off_flag = False

        if (    
            not avp_command_listener.initiate_parking and
            not drop_off_completed and
            avp_command_listener.ego_x is not None and
            is_in_drop_off_zone(avp_command_listener.ego_x, avp_command_listener.ego_y)
        ):
            # print("Drop-off valet destination reached.")
            avp_command_listener.status_publisher.publish(String(data="Arrived at drop-off area."))
            avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Arrived at drop-off area."))


        # motion_state_subscriber.get_logger().info(f"[DEBUG] Motion Status: {motion_state_subscriber.state}")

        if  (
            motion_state_subscriber.state == 1 and 
            not drop_off_completed and
            avp_command_listener.ego_x is not None and
            is_in_drop_off_zone(avp_command_listener.ego_x, avp_command_listener.ego_y)
        ):  
            
            
            if avp_command_listener.current_queue:

                if str(avp_command_listener.current_queue[0]) != str(avp_command_listener.vehicle_id):

                    avp_command_listener.get_logger().info(f"[DEBUG] Vehicle is NOT first in queue — waiting for vehicle ahead.")
                    
                    avp_command_listener.status_publisher.publish(String(data="Waiting 10 seconds for vehicle ahead..."))
                    avp_command_listener.status_update_publisher.publish(String(
                            data=f"{avp_command_listener.vehicle_id}:Waiting 10 seconds for vehicle ahead..."))
                    time.sleep(2)

                    first_in_line = avp_command_listener.current_queue[0]

                    first_status_of_first_in_line = avp_command_listener.status_all_data.get(str(first_in_line), "")

                    waited = 0
                    front_car_moved = False

                    while waited < 10:
                        loop_start = time.time()

                        rclpy.spin_once(avp_command_listener, timeout_sec=0.1)
                        time.sleep(0.05)

                        next_status_of_first_in_line = avp_command_listener.status_all_data.get(str(first_in_line), "")

                        avp_command_listener.get_logger().info(f"Vehicle {first_in_line}'s status after {waited+1} sec: {next_status_of_first_in_line}")

                        if next_status_of_first_in_line != first_status_of_first_in_line:
                            avp_command_listener.get_logger().info(f"Status changed! {first_status_of_first_in_line} → {next_status_of_first_in_line}")
                            if ("Autonomous valet parking started" in next_status_of_first_in_line or "Waiting for an available parking spot..." in next_status_of_first_in_line):
                                front_car_moved = True
                                break
                                
                        first_status_of_first_in_line = next_status_of_first_in_line
                        waited += 1

                        time_to_wait = 1.0 - (time.time() - loop_start)
                        if time_to_wait > 0:
                            time.sleep(time_to_wait)

                    # Final spin to catch any last-millisecond updates
                    rclpy.spin_once(avp_command_listener, timeout_sec=0.2)
                    final_status_of_first_in_line = avp_command_listener.status_all_data.get(str(first_in_line), "")

                    if final_status_of_first_in_line != first_status_of_first_in_line:
                        avp_command_listener.get_logger().info(f"[FINAL CHECK] Status changed! {first_status_of_first_in_line} → {final_status_of_first_in_line}")
                        if ("Autonomous valet parking started" in final_status_of_first_in_line or "Waiting for an available parking spot..." in final_status_of_first_in_line):
                            front_car_moved = True

                    if front_car_moved:
                        avp_command_listener.status_publisher.publish(String(data="Vehicle ahead is preparing to leave."))
                        avp_command_listener.status_update_publisher.publish(String(
                            data=f"{avp_command_listener.vehicle_id}:Vehicle ahead preparing to leave."))
                        time.sleep(2)

                        avp_command_listener.status_publisher.publish(String(data="Moving up in queue..."))
                        avp_command_listener.status_update_publisher.publish(
                            String(data=f"{avp_command_listener.vehicle_id}:Moving up in queue...")
                        )
                        
                        # Wait until car has fully arrived at start of drop off zone
                        while route_state_subscriber.state != 6:
                            rclpy.spin_once(route_state_subscriber, timeout_sec=0.2)
                        
                        avp_command_listener.handle_owner_exit()
                        drop_off_completed = True

                    else:
                        avp_command_listener.status_publisher.publish(String(data="Vehicle(s) ahead are still stationary. Proceeding with drop-off."))
                        avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Vehicle(s) ahead are still stationary. Proceeding with drop-off."))
                        time.sleep(2)
                        avp_command_listener.handle_owner_exit()
                        drop_off_completed = True

                    
                else:
                    avp_command_listener.get_logger().info(f"[DEBUG] Vehicle IS first in queue — proceeding with drop-off.")
                    
                    avp_command_listener.handle_owner_exit()
                    drop_off_completed = True

            else:
                avp_command_listener.get_logger().warn(f"[WARN] Queue is empty!")

                if args.debug:
                    avp_command_listener.handle_owner_exit()
                    drop_off_completed = True

        if avp_command_listener.start_avp and not avp_command_listener.initiate_parking: 
            # start_avp_clicked = True
            avp_command_listener.status_publisher.publish(String(data="Autonomous valet parking started..."))
            avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Autonomous valet parking started..."))
            time.sleep(2)

            avp_command_listener.initiate_parking = True

        if avp_command_listener.initiate_parking and not parking_complete:

            rclpy.spin_once(parking_spot_subscriber, timeout_sec=0.1)  

            parking_spots = parking_spot_subscriber.available_parking_spots

            if parking_spots is None:
                avp_command_listener.status_publisher.publish(String(data="Waiting for an available parking spot..."))
                avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Waiting for an available parking spot..."))
                time.sleep(2)

            if parking_spots is not None:
                first_spot_in_queue = int(parking_spots.strip().strip('[]').split(',')[0].strip())

                if first_spot_in_queue != chosen_parking_spot:
                    if counter == 1:
                        print('\nParking spot #%s was taken.' % chosen_parking_spot)
                        counter = 0

                    # Printing the first value
                    print('\nAvailable parking spot found: %s' % first_spot_in_queue)

                    avp_command_listener.status_publisher.publish(String(data="Available parking spot found."))
                    avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Available parking spot found."))
                    time.sleep(2)
                    avp_command_listener.status_publisher.publish(String(data=f"Parking in Spot {first_spot_in_queue}."))
                    avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Parking in Spot {first_spot_in_queue}."))

                    parking_spot_goal_pose_command = parking_spot_locations[first_spot_in_queue]
                    run_ros2_command(parking_spot_goal_pose_command)
                    run_ros2_command(engage_auto_mode)

                    queue_msg = String()
                    queue_msg.data = avp_command_listener.vehicle_id
                    avp_command_listener.queue_remove_pub.publish(queue_msg)
                    print(f"[QUEUE] Sent queue removal request for {queue_msg.data}")

                    if first_spot_in_queue not in avp_command_listener.reserved_spots_list:
                        msg = String()
                        msg.data = str(first_spot_in_queue)
                        avp_command_listener.reserved_spot_request_pub.publish(msg)
                        avp_command_listener.reserved_spots_list.append(first_spot_in_queue)
                        print(f"[AVP] Sent reservation request: {msg.data}")
                    else:
                        print(f"[AVP] Spot {first_spot_in_queue} already requested.")

                    route_state_subscriber.state = -1

                    print('Setting goal pose to parking spot:', first_spot_in_queue)

                    counter += 1
                    chosen_parking_spot = first_spot_in_queue

                    parking_complete = True
                
            time.sleep(1)

        if route_state_subscriber.state == 6 and parking_complete and not reserved_cleared:
            avp_command_listener.status_publisher.publish(String(data="Car has been parked."))
            avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Car has been parked."))

            msg = String()
            msg.data = str(chosen_parking_spot)
            avp_command_listener.reserved_spot_remove_pub.publish(msg)
            print(f"[AVP] Sent removal request: {msg.data}")

            reserved_cleared = True

            route_state_subscriber.state = -1

        if avp_command_listener.retrieve_vehicle and not retrieve_vehicle_complete:
            print("Going to Drop Off Zone.")
            avp_command_listener.status_publisher.publish(String(data="Car is called for retrieval."))
            avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Car is called for retrieval."))
            run_ros2_command(retrieve_vehicle_goal_pose)
            run_ros2_command(engage_auto_mode)
            retrieve_vehicle_complete = True
        
        if route_state_subscriber.state == 6 and retrieve_vehicle_complete and left_for_new_destination:
                avp_command_listener.status_publisher.publish(String(data="Car has been retrieved."))
                avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Car has been retrieved."))
                time.sleep(2)
                avp_command_listener.status_publisher.publish(String(data="Owner is getting in..."))
                avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Owner is getting in..."))
                time.sleep(2)
                avp_command_listener.status_publisher.publish(String(data="Owner is inside."))
                avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Owner is inside."))
                time.sleep(2)
                avp_command_listener.status_publisher.publish(String(data="Leaving to new destination."))
                avp_command_listener.status_update_publisher.publish(String(data=f"{avp_command_listener.vehicle_id}:Leaving to new destination."))
                run_ros2_command(leave_area_goal_pose)
                left_for_new_destination = True

        rclpy.spin_once(route_state_subscriber, timeout_sec=1)
        rclpy.spin_once(motion_state_subscriber, timeout_sec=1)
        rclpy.spin_once(avp_command_listener, timeout_sec=1)

    route_state_subscriber.destroy_node()
    motion_state_subscriber.destroy_node()
    parking_spot_subscriber.destroy_node()
    avp_command_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()