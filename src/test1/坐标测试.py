#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose # Import Pose to store just the pose part if needed, though Odometry's pose.pose is Pose
from nav_msgs.msg import Odometry # Import Odometry message type
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from apriltag_ros.msg import AprilTagDetectionArray

current_state = State()
current_odom = None # Use current_odom again to store the Odometry message
# current_pose is effectively part of current_odom
detected_tags = {}  # {id: (x, y, z)} in camera frame
# Pin Definitions
output_pin = 18  # BCM pin 18, BOARD pin 12   
landing_tag={}

def pose_cb(msg: Odometry): # Parameter type hint back to Odometry
    global current_odom
    current_odom = msg
     # Store the Odometry message
    # print(current_odom.pose.pose.position) # Example of accessing position
    # Remove the print(current_pose) as current_pose is not the main variable now

def tag_detections_cb(msg):
    # Add this line to confirm callback is triggered
    
    global detected_tags
   
    for det in msg.detections:
        
        tag_id = det.id[0]
        position = det.pose.pose.pose.position
        detected_tags[tag_id] = (position.x, position.y, position.z)

    

def adjust_pose_for_apriltag(pose, tag_offset):
    # tag_offset: 相机坐标系下的坐标 (右向x, 下向y, 前向z)
    # 将tag偏移转换到无人机坐标系（通常与机体系一致），再调整pose
    dx, dy, dz = tag_offset
    pose.pose.position.x += 0
    pose.pose.position.y -= dx
    pose.pose.position.z -= dy
   
    return pose

def landing_tag_callback(msg):
    global landing_tag
    if not msg.detections:
        
        return

    for det in msg.detections:
        tag_id = det.id[0]
        position = det.pose.pose.pose.position
        landing_tag[tag_id] = (position.x, position.y, position.z)
        rospy.loginfo(f"Landing tag {tag_id} detected at: {landing_tag[tag_id]}")
    
    landing_tag[tag_id] =(position.x,position.y,position.z)

def adjust_pose_for_landing(pose,offset):
    dx,dy,dz = offset
    pose.pose.position.x -= dx+0.08
    pose.pose.position.y += dy
    pose.pose.position.z -= dz+0.05
    
    return pose
if __name__ == "__main__":
    rospy.init_node("offboard_apriltag_node")
    #state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    # Subscribe to the Odometry topic with Odometry message type
    current_pose_sub = rospy.Subscriber("/vins_fusion/imu_propagate", Odometry, callback=pose_cb) # Message type is Odometry
    tag_sub = rospy.Subscriber("/d455/tag_detections", AprilTagDetectionArray, callback=tag_detections_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    tag_sub_usb = rospy.Subscriber("/usb_cam/tag_detections",AprilTagDetectionArray,landing_tag_callback)
    rate = rospy.Rate(20)

    # Wait for connection (assuming state_cb is still needed or will be re-enabled)
    # while not rospy.is_shutdown() and not current_state.connected:
    #     rate.sleep()

    # Wait for the first Odometry message
    rospy.loginfo("Waiting for Odometry message...")
    while not rospy.is_shutdown() and current_odom is None:
        rate.sleep()
    rospy.loginfo("Received first Odometry message.")

    # 初始化航点列表和每个航点对应tag_id
    waypoints = [
        {"pos": (0,0, 0), "tag_id": 1},
        {"pos": (0,-0.75, 1.4), "tag_id": 2},
        {"pos": (0,-1.25, 1.4), "tag_id": 3},
        {"pos": (0,-1.25, 1.0), "tag_id": 4},
        {"pos": (0,-1.75, 1.0), "tag_id": 5},
        {"pos": (0,-1.75, 1.4), "tag_id": 6},
    ]

    pose = PoseStamped()
    wp_index = 0
    reached_wp = False

    # Main control loop
    while not rospy.is_shutdown():

        if  0==1:#wp_index < len(waypoints):
            wp = waypoints[wp_index]
            # For simplicity and to allow initial movement, set target pose outside tag detection logic
            pose.pose.position.x = wp["pos"][0]
            pose.pose.position.y = wp["pos"][1]
            pose.pose.position.z = wp["pos"][2]

            # Only proceed if current_odom is not None (should be true after initial wait, but good practice)
            if current_odom is not None:
                dx = current_odom.pose.pose.position.x - pose.pose.position.x
                dy = current_odom.pose.pose.position.y - pose.pose.position.y
                dz = current_odom.pose.pose.position.z - pose.pose.position.z
                dist = math.sqrt(dx**2 + dy**2 + dz**2)

                # Revert condition to check distance and reached_wp flag
                if dist < 0.3 and not reached_wp:
                    rospy.loginfo(f"到达航点 {wp_index + 1}，等待 AprilTag {wp['tag_id']} 检测")

                    # Check for tag detection without a fixed time limit loop here
                    # This will check if the tag is detected in the current or recent AprilTag message
                    if wp["tag_id"] in detected_tags:
                        offset = detected_tags[wp["tag_id"]]
                        rospy.loginfo(f"Detected tag {wp['tag_id']}, offset: {offset}")
                        # rospy.loginfo(f"检测到 tag {wp['tag_id']}，偏移: {offset}") # Duplicate log

                        # Create a temporary Pose object from the current odometry pose
                        # and adjust it based on the AprilTag offset
                        temp_pose = PoseStamped()
                        temp_pose.pose.position.x = current_odom.pose.pose.position.x
                        temp_pose.pose.position.y = current_odom.pose.pose.position.y
                        temp_pose.pose.position.z = current_odom.pose.pose.position.z
                        # Assuming adjust_pose_for_apriltag modifies the pose in place and returns it
                        adjusted_temp_pose = adjust_pose_for_apriltag(temp_pose, offset)

                        # Update the target pose (PoseStamped variable) with the adjusted position
                        pose.pose.position.x = adjusted_temp_pose.pose.position.x
                        pose.pose.position.y = adjusted_temp_pose.pose.position.y
                        pose.pose.position.z = adjusted_temp_pose.pose.position.z

                        rospy.loginfo(f"Adjusted target pose position based on current odom: x={pose.pose.position.x}, y={pose.pose.position.y}, z={pose.pose.position.z}")
                        # print(pose) # Keep user's print if they added it

                        # Move to next waypoint after tag detection
                        wp_index += 1
                        reached_wp = True # Mark as reached after processing tag
                        rospy.loginfo(f"Moving to next waypoint: {wp_index + 1}")
                    else:
                        rospy.loginfo(f"Tag {wp['tag_id']} not detected yet.")

                # Reset reached_wp flag once drone moves significantly away from the current target
                # This prevents re-triggering tag logic at the same waypoint
                if dist >= 0.5: # Use a slightly larger threshold to avoid flapping near the waypoint
                    reached_wp = False

            # Publish the current target pose regardless of tag detection status at this point
            # The pose is either the original waypoint or adjusted if tag was detected in a previous iteration
            local_pos_pub.publish(pose)

        else: # All waypoints visited rospy.loginfo("所有航点完成,准备返航")
            pose.pose.position.x=0
            pose.pose.position.y=0
            pose.pose.position.z=0.3
            if math.sqrt(current_odom.pose.pose.position.x**2+current_odom.pose.pose.position.y**2) <= 0.2 :
                if 0 in landing_tag:
                    offset = landing_tag[0]
                       
                    temp_pose = PoseStamped()
                    temp_pose.pose.position.x = current_odom.pose.pose.position.x
                    temp_pose.pose.position.y = current_odom.pose.pose.position.y
                    temp_pose.pose.position.z = current_odom.pose.pose.position.z
                    print(temp_pose)
                        # Assuming adjust_pose_for_apriltag modifies the pose in place and returns it
                    adjusted_temp_pose = adjust_pose_for_landing(temp_pose, offset)
                    
                        # Update the target pose (PoseStamped variable) with the adjusted position
                    pose.pose.position.x = adjusted_temp_pose.pose.position.x
                    pose.pose.position.y = adjusted_temp_pose.pose.position.y
                    pose.pose.position.z = adjusted_temp_pose.pose.position.z
                    print(pose)
                    exit(0)
                else:
                    rospy.loginfo("Landing tag ID 0 not detected yet.")










            # Logic for landing tag (if applicable)
            # Ensure landing_tag is populated before using it
            # This part needs careful review based on how landing_tag_callback works
            # if math.sqrt(current_odom.pose.pose.position.x**2+current_odom.pose.pose.position.y**2) <= 0.2:
            #     offset = landing_tag # Assuming landing_tag structure is compatible
            #     pose=adjust_pose_for_landing(pose,offset)
            #     # break # This break would exit the main while l
            # Consider adding a condition to exit the main loop after landing or returning home
            # e.g., if current_odom.pose.pose.position is very close to home for a duration

        # Sleep to control the loop frequency
        rate.sleep()
