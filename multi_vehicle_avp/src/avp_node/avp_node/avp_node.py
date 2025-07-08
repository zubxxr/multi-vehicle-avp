import rclpy
from std_msgs.msg import String
import argparse
from enum import Enum, auto
import time

from vehicle.state_subscribers import RouteStateSubscriber, MotionStateSubscriber, ParkingSpotSubscriber
from vehicle.command_listener import AVPCommandListener
from utils.ros_helpers import run_ros2_command, build_ros2_pub, wait_until_route_complete
from utils.dropoff_and_parking import parking_spot_locations, is_in_drop_off_zone

class AVPState(Enum):
    INIT = auto()                     
    WAIT_FOR_DROP_OFF = auto()      
    HANDLE_DROP_OFF = auto()        
    START_AVP = auto()              
    PARK_VEHICLE = auto()           
    RESERVE_SPOT = auto()           
    CLEAR_RESERVATION = auto()      
    RETRIEVE = auto()               
    FINALIZE_RETRIEVAL = auto()     
    COMPLETE = auto()               


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

    engage_autonomous_mode = build_ros2_pub("/autoware/engage", "autoware_vehicle_msgs/msg/Engage", "{engage: True}")
    disengage_autonomous_mode = build_ros2_pub("/autoware/engage", "autoware_vehicle_msgs/msg/Engage", "{engage: False}")

    timeout = 15
    if args.debug: # if debug is true, that means planning simulator is used, meaning less traffic, so 5 is sufficient
        timeout = 5
    else: 
        timeout = 15 # if using e2e simulator, there is more traffic, so timeout of 15 is needed.
    
    while avp_command_listener.vehicle_count_request_pub.get_subscription_count() == 0 and timeout > 0:
        avp_command_listener.get_logger().info("[INFO] Waiting for /avp/vehicle_count/request subscriber...")
        time.sleep(1)
        timeout -= 1

    avp_command_listener.vehicle_id_pub.publish(String(data=str(avp_command_listener.vehicle_id)))
    avp_command_listener.vehicle_count_request_pub.publish(String(data="add_me"))

    current_state = AVPState.INIT
    chosen_parking_spot = None
    front_car_moved = False

    counter = 0
    waiting_for_queue = True

    if args.debug:
        run_ros2_command(set_initial_pose)
    
    while rclpy.ok():
        rclpy.spin_once(avp_command_listener, timeout_sec=0.5)
        rclpy.spin_once(route_state_subscriber, timeout_sec=0.5)
        rclpy.spin_once(motion_state_subscriber, timeout_sec=0.5)

        if current_state == AVPState.INIT:
            avp_command_listener.publish_vehicle_status("Arrived at location.")
            current_state = AVPState.WAIT_FOR_DROP_OFF

        elif current_state == AVPState.WAIT_FOR_DROP_OFF:
            if avp_command_listener.head_to_drop_off:
                run_ros2_command(head_to_drop_off)
                run_ros2_command(engage_autonomous_mode)
                current_state = AVPState.HANDLE_DROP_OFF

        elif current_state == AVPState.HANDLE_DROP_OFF:
            if avp_command_listener.ego_x and is_in_drop_off_zone(avp_command_listener.ego_x, avp_command_listener.ego_y):
                avp_command_listener.publish_vehicle_status("Arrived at drop-off area.")

                if avp_command_listener.current_queue:
                    if str(avp_command_listener.current_queue[0]) == str(avp_command_listener.vehicle_id):
                        wait_until_route_complete(route_state_subscriber)
                        avp_command_listener.handle_owner_exit()
                        current_state = AVPState.START_AVP

                    else:
                        while motion_state_subscriber.state != 1:
                            rclpy.spin_once(motion_state_subscriber, timeout_sec=0.2)

                        avp_command_listener.get_logger().info(f"[DEBUG] Vehicle is NOT first in queue — waiting for vehicle ahead.")
                        avp_command_listener.publish_vehicle_status("Waiting 10 seconds for vehicle ahead...")

                        first_in_line = avp_command_listener.current_queue[0]
                        
                        start_time = time.time()

                        while True:
                            rclpy.spin_once(avp_command_listener, timeout_sec=0.2)
                            status = avp_command_listener.all_vehicle_status.get(str(first_in_line), "")

                            if status and "On standby..." not in status:
                                front_car_moved = True
                                break
                            elif time.time() - start_time > 10:
                                avp_command_listener.publish_vehicle_status(f"Waiting... {start_time} sec")   
                                break

                        if front_car_moved:
                            avp_command_listener.publish_vehicle_status("Vehicle ahead is preparing to leave.")
                            time.sleep(2)

                            avp_command_listener.publish_vehicle_status("Moving up in queue...")
                            
                            # Wait until car has fully arrived at start of drop off zone
                            wait_until_route_complete(route_state_subscriber)
                            
                            avp_command_listener.handle_owner_exit()
                            current_state = AVPState.START_AVP

                        else:
                            avp_command_listener.publish_vehicle_status("Vehicle(s) ahead are still stationary. Proceeding with drop-off.")
                            time.sleep(2)
                            run_ros2_command(disengage_autonomous_mode)
                            avp_command_listener.handle_owner_exit(waiting_for_queue)
                            run_ros2_command(engage_autonomous_mode)
                            avp_command_listener.publish_vehicle_status("Waiting to proceed in queue...")
                            current_state = AVPState.START_AVP
                else:
                    avp_command_listener.get_logger().warn(f"[WARN] Queue is empty!")
                    wait_until_route_complete(route_state_subscriber)
                    if args.debug:
                        avp_command_listener.handle_owner_exit()
                        current_state = AVPState.START_AVP

        elif current_state == AVPState.START_AVP:
            wait_until_route_complete(route_state_subscriber)
            avp_command_listener.publish_vehicle_status("On standby...")
            if avp_command_listener.start_avp:
                avp_command_listener.publish_vehicle_status("Autonomous valet parking started...")
                avp_command_listener.initiate_parking = True
                current_state = AVPState.PARK_VEHICLE

        elif current_state == AVPState.PARK_VEHICLE:
            rclpy.spin_once(parking_spot_subscriber, timeout_sec=0.5)
            parking_spots = parking_spot_subscriber.available_parking_spots
            if parking_spots:
                first_spot_in_queue = int(parking_spots.strip('[]').split(',')[0])
                if first_spot_in_queue != chosen_parking_spot:
                    chosen_parking_spot = first_spot_in_queue

                    avp_command_listener.get_logger().info('\nAvailable parking spot found: %s' % first_spot_in_queue)
                    avp_command_listener.publish_vehicle_status("Available parking spot found.")
                    time.sleep(2)

                    avp_command_listener.publish_vehicle_status(f"Parking in Spot {first_spot_in_queue}.")
                    run_ros2_command(parking_spot_locations[first_spot_in_queue])
                    run_ros2_command(engage_autonomous_mode)

                    avp_command_listener.queue_remove_pub.publish(String(data=avp_command_listener.vehicle_id))

                    current_state = AVPState.RESERVE_SPOT

        elif current_state == AVPState.RESERVE_SPOT:
            if first_spot_in_queue not in avp_command_listener.reserved_spots_list:
                avp_command_listener.reserved_spot_request_pub.publish(String(data=str(first_spot_in_queue)))
                avp_command_listener.reserved_spots_list.append(first_spot_in_queue)
                avp_command_listener.get_logger().info(f"[AVP] Sent reservation request: {str(first_spot_in_queue)}")
            else:
                avp_command_listener.get_logger().warn(f"[AVP] Spot {str(first_spot_in_queue)} already requested.")

            chosen_parking_spot = first_spot_in_queue
            current_state = AVPState.CLEAR_RESERVATION
            route_state_subscriber.state = -1

        elif current_state == AVPState.CLEAR_RESERVATION:
            wait_until_route_complete(route_state_subscriber)

            avp_command_listener.publish_vehicle_status("Car has been parked.")
            avp_command_listener.reserved_spot_remove_pub.publish(String(data=str(chosen_parking_spot)))
            current_state = AVPState.RETRIEVE

        elif current_state == AVPState.RETRIEVE:
            if avp_command_listener.retrieve_vehicle:
                avp_command_listener.publish_vehicle_status("Car is called for retrieval.")
                run_ros2_command(retrieve_vehicle_goal_pose)
                run_ros2_command(engage_autonomous_mode)
                current_state = AVPState.FINALIZE_RETRIEVAL
                route_state_subscriber.state = -1

        elif current_state == AVPState.FINALIZE_RETRIEVAL:
            wait_until_route_complete(route_state_subscriber)

            avp_command_listener.publish_vehicle_status("Car has been retrieved.")
            time.sleep(2)
            avp_command_listener.publish_vehicle_status("Owner is getting in...")
            time.sleep(5)
            avp_command_listener.publish_vehicle_status("Owner is inside.")
            time.sleep(2)
            avp_command_listener.publish_vehicle_status("Leaving for new destination.")
            run_ros2_command(leave_area_goal_pose)
            run_ros2_command(engage_autonomous_mode)
            current_state = AVPState.COMPLETE
            route_state_subscriber.state = -1

        elif current_state == AVPState.COMPLETE:
            time.sleep(3)
            wait_until_route_complete(route_state_subscriber)
            
            avp_command_listener.publish_vehicle_status("AVP process complete.")
            break

    avp_command_listener.destroy_node()
    route_state_subscriber.destroy_node()
    motion_state_subscriber.destroy_node()
    parking_spot_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
