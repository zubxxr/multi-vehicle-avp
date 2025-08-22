import rclpy
from std_msgs.msg import String
import argparse
from enum import Enum, auto
import time

from vehicle.state_subscribers import RouteStateSubscriber, MotionStateSubscriber, ParkingSpotSubscriber
from vehicle.command_listener import AVPCommandListener
from utils.ros_helpers import run_ros2_command, build_ros2_pub, wait_until_route_complete
from utils.dropoff_and_parking import parking_spot_locations, is_in_drop_off_zone

class VehicleState(Enum):
    # Arrival & drop-off phase
    ARRIVED_AT_LOCATION = auto()      
    AWAITING_DROPOFF_COMMAND = auto()
    NAVIGATING_TO_DROPOFF = auto()               
    ARRIVED_AT_DROPOFF = auto()          

    # Queue phase        
    CHECK_FOR_VEHICLES_IN_QUEUE = auto()         
    WAITING_FOR_VEHICLES_IN_QUEUE = auto()     
    QUEUE_INACTIVE_OWNER_DROPOFF = auto()   
    MOVING_UP_IN_QUEUE = auto()                   
    WAITING_IN_QUEUE = auto()
    DROPPING_OFF_OWNER = auto()                   
    DROPPING_OFF_OWNER_POST_QUEUE = auto()

    # AVP start
    WAITING_FOR_AVP_START = auto()           
    AVP_STARTED = auto()           

    # Parking flow
    FINDING_PARKING_SPOT = auto()                               
    RESERVING_PARKING_SPOT = auto()                    
    DRIVING_TO_PARKING_SPOT = auto()                         
    PARKED_AND_CLEARING_RESERVATION = auto()                  

    # Retrieval phase
    WAITING_FOR_RETRIEVAL = auto()               
    CALLED_FOR_RETRIEVAL = auto()                 
    DRIVING_TO_RETRIEVAL_POINT = auto()            
    PICKING_UP_OWNER = auto()    
    LEAVING_AREA = auto()    

    # Final
    AVP_COMPLETE = auto()     
                    
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

    # send request to register vehicle into count
    avp_command_listener.vehicle_id_pub.publish(String(data=str(avp_command_listener.vehicle_id)))
    avp_command_listener.vehicle_count_request_pub.publish(String(data="add_me"))

    current_state = VehicleState.ARRIVED_AT_LOCATION
    chosen_parking_spot = None
    front_car_moved = False
    first_spot_in_empty_parking_spot_list = -1
    waiting_for_queue = True

    if args.debug:
        run_ros2_command(set_initial_pose)
        
    try: 
        while rclpy.ok():
            rclpy.spin_once(avp_command_listener, timeout_sec=0.5)
            rclpy.spin_once(route_state_subscriber, timeout_sec=0.5)
            rclpy.spin_once(motion_state_subscriber, timeout_sec=0.5)
            
            if current_state == VehicleState.ARRIVED_AT_LOCATION:
                avp_command_listener.publish_vehicle_status("Arrived at location.")
                current_state = VehicleState.AWAITING_DROPOFF_COMMAND
            
            elif current_state == VehicleState.AWAITING_DROPOFF_COMMAND:
                # if "Drop-off" clicked in the AVPPanel in Autoware
                if avp_command_listener.head_to_drop_off:
                    run_ros2_command(head_to_drop_off)
                    run_ros2_command(engage_autonomous_mode)
                    current_state = VehicleState.NAVIGATING_TO_DROPOFF

            elif current_state == VehicleState.NAVIGATING_TO_DROPOFF:
                # if the vehicle is in the drop off location
                if avp_command_listener.ego_x and is_in_drop_off_zone(avp_command_listener.ego_x, avp_command_listener.ego_y):
                    current_state = VehicleState.ARRIVED_AT_DROPOFF

            elif current_state == VehicleState.ARRIVED_AT_DROPOFF:
                avp_command_listener.publish_vehicle_status("Arrived at drop-off area.")
                current_state = VehicleState.CHECK_FOR_VEHICLES_IN_QUEUE

            elif current_state == VehicleState.CHECK_FOR_VEHICLES_IN_QUEUE:
                if avp_command_listener.current_queue:
                    if str(avp_command_listener.current_queue[0]) == str(avp_command_listener.vehicle_id):
                        current_state = VehicleState.DROPPING_OFF_OWNER
                    else:
                        current_state = VehicleState.WAITING_FOR_VEHICLES_IN_QUEUE
                else:
                    avp_command_listener.get_logger().warn(f"[WARN] Queue is empty!")
                    if args.debug:
                        current_state = VehicleState.DROPPING_OFF_OWNER

            elif current_state == VehicleState.WAITING_FOR_VEHICLES_IN_QUEUE:
                while motion_state_subscriber.state != 1:
                    rclpy.spin_once(motion_state_subscriber, timeout_sec=0.2)

                avp_command_listener.get_logger().info(f"[DEBUG] Vehicle is NOT first in queue — waiting for vehicle ahead.")
                avp_command_listener.publish_vehicle_status("Waiting 10 seconds for vehicle ahead...")
                time.sleep(2)

                first_in_line = avp_command_listener.current_queue[0]
                start_time = time.time()

                while True:
                    rclpy.spin_once(avp_command_listener, timeout_sec=0.2)
                    waited_time = int(time.time() - start_time) 
                    avp_command_listener.publish_vehicle_status(f"Waiting... {waited_time} sec")

                    status = avp_command_listener.all_vehicle_status.get(str(first_in_line), "")

                    if status and "Autonomous valet parking started..." in status:
                        front_car_moved = True
                        break
                    elif waited_time >= 10:
                        avp_command_listener.publish_vehicle_status(f"Waited 10 seconds. Proceeding with drop-off.")
                        time.sleep(2)
                        break

                if front_car_moved:
                    current_state = VehicleState.MOVING_UP_IN_QUEUE
                else:
                    current_state = VehicleState.QUEUE_INACTIVE_OWNER_DROPOFF

            elif current_state == VehicleState.MOVING_UP_IN_QUEUE:
                avp_command_listener.publish_vehicle_status("Vehicle ahead is preparing to leave.")
                time.sleep(2)
                avp_command_listener.publish_vehicle_status("Moving up in queue...")
                current_state = VehicleState.DROPPING_OFF_OWNER

            elif current_state == VehicleState.DROPPING_OFF_OWNER:
                wait_until_route_complete(route_state_subscriber)
                avp_command_listener.handle_owner_exit()
                current_state = VehicleState.WAITING_FOR_AVP_START

            elif current_state == VehicleState.QUEUE_INACTIVE_OWNER_DROPOFF:
                avp_command_listener.publish_vehicle_status("Vehicle(s) ahead are still stationary. Proceeding with drop-off.")
                current_state = VehicleState.DROPPING_OFF_OWNER_POST_QUEUE

            elif current_state == VehicleState.DROPPING_OFF_OWNER_POST_QUEUE:
                time.sleep(2)
                run_ros2_command(disengage_autonomous_mode)
                avp_command_listener.handle_owner_exit(waiting_for_queue)
                run_ros2_command(engage_autonomous_mode)
                avp_command_listener.publish_vehicle_status("Waiting to proceed in queue...")
                current_state = VehicleState.WAITING_IN_QUEUE
                
            elif current_state == VehicleState.WAITING_IN_QUEUE:
                wait_until_route_complete(route_state_subscriber)
                avp_command_listener.publish_vehicle_status("On standby...")
                current_state = VehicleState.WAITING_FOR_AVP_START
                
            elif current_state == VehicleState.WAITING_FOR_AVP_START:
                if avp_command_listener.start_avp:
                    current_state = VehicleState.AVP_STARTED

            elif current_state == VehicleState.AVP_STARTED:
                avp_command_listener.publish_vehicle_status("Autonomous valet parking started...")
                time.sleep(2)
                current_state = VehicleState.FINDING_PARKING_SPOT

            elif current_state == VehicleState.FINDING_PARKING_SPOT:
                rclpy.spin_once(parking_spot_subscriber, timeout_sec=0.5)
                parking_spots = parking_spot_subscriber.available_parking_spots
                if parking_spots:
                    first_spot_in_empty_parking_spot_list = int(parking_spots.strip('[]').split(',')[0])
                    if first_spot_in_empty_parking_spot_list != chosen_parking_spot:
                        chosen_parking_spot = first_spot_in_empty_parking_spot_list

                        avp_command_listener.get_logger().info('\nAvailable parking spot found: %s' % first_spot_in_empty_parking_spot_list)
                        avp_command_listener.publish_vehicle_status("Available parking spot found.")
                        time.sleep(2)

                        current_state = VehicleState.RESERVING_PARKING_SPOT
                else:
                    avp_command_listener.get_logger().warn("[WARN] No available parking spots received from the subscriber.")

            elif current_state == VehicleState.RESERVING_PARKING_SPOT:
                if first_spot_in_empty_parking_spot_list not in avp_command_listener.reserved_spots_list:
                    avp_command_listener.reserved_spot_request_pub.publish(String(data=str(first_spot_in_empty_parking_spot_list)))
                    avp_command_listener.reserved_spots_list.append(first_spot_in_empty_parking_spot_list)
                    avp_command_listener.get_logger().info(f"[AVP] Sent reservation request: {str(first_spot_in_empty_parking_spot_list)}")
                else:
                    avp_command_listener.get_logger().warn(f"[AVP] Spot {str(first_spot_in_empty_parking_spot_list)} already requested.")

                chosen_parking_spot = first_spot_in_empty_parking_spot_list
                route_state_subscriber.state = -1

                current_state = VehicleState.DRIVING_TO_PARKING_SPOT

            elif current_state == VehicleState.DRIVING_TO_PARKING_SPOT:
                avp_command_listener.publish_vehicle_status(f"Parking in Spot {first_spot_in_empty_parking_spot_list}.")
                avp_command_listener.queue_remove_pub.publish(String(data=avp_command_listener.vehicle_id))
            
                run_ros2_command(parking_spot_locations[first_spot_in_empty_parking_spot_list])
                run_ros2_command(engage_autonomous_mode)
                wait_until_route_complete(route_state_subscriber)
                current_state = VehicleState.PARKED_AND_CLEARING_RESERVATION

            elif current_state == VehicleState.PARKED_AND_CLEARING_RESERVATION:
                avp_command_listener.publish_vehicle_status("Car has been parked.")
                avp_command_listener.reserved_spot_remove_pub.publish(String(data=str(chosen_parking_spot)))
                current_state = VehicleState.WAITING_FOR_RETRIEVAL

            elif current_state == VehicleState.WAITING_FOR_RETRIEVAL:
                if avp_command_listener.retrieve_vehicle:
                    current_state = VehicleState.CALLED_FOR_RETRIEVAL

            elif current_state == VehicleState.CALLED_FOR_RETRIEVAL:
                avp_command_listener.publish_vehicle_status("Car is called for retrieval.") 
                current_state = VehicleState.DRIVING_TO_RETRIEVAL_POINT

            elif current_state == VehicleState.DRIVING_TO_RETRIEVAL_POINT:
                run_ros2_command(retrieve_vehicle_goal_pose)
                run_ros2_command(engage_autonomous_mode)
                route_state_subscriber.state = -1
                wait_until_route_complete(route_state_subscriber)
                current_state = VehicleState.PICKING_UP_OWNER

            elif current_state == VehicleState.PICKING_UP_OWNER:
                avp_command_listener.publish_vehicle_status("Car has been retrieved.")
                time.sleep(2)
                avp_command_listener.publish_vehicle_status("Owner is getting in...")
                time.sleep(5)
                avp_command_listener.publish_vehicle_status("Owner is inside.")
                time.sleep(2)
                current_state = VehicleState.LEAVING_AREA

            elif current_state == VehicleState.LEAVING_AREA:
                avp_command_listener.publish_vehicle_status("Leaving for new destination.")
                run_ros2_command(leave_area_goal_pose)
                run_ros2_command(engage_autonomous_mode)
                route_state_subscriber.state = -1
                time.sleep(3)
                wait_until_route_complete(route_state_subscriber)
                current_state = VehicleState.AVP_COMPLETE

            elif current_state == VehicleState.AVP_COMPLETE:
                avp_command_listener.publish_vehicle_status("AVP process complete.")
                break

    except KeyboardInterrupt:
        avp_command_listener.get_logger().info("KeyboardInterrupt detected. Shutting down gracefully...")
    except Exception as e:
        avp_command_listener.get_logger().error(f"Unexpected error: {e}")
    finally:
        avp_command_listener.destroy_node()
        route_state_subscriber.destroy_node()
        motion_state_subscriber.destroy_node()
        parking_spot_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()