#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from apriltag_ros.msg import AprilTagDetectionArray  
import RPi.GPIO as GPIO
import time


current_state = State()
current_pose = PoseStamped()
detected_tags = {}  # {id: (x, y, z)} in camera frame
# Pin Definitions
output_pin = 18  # BCM pin 18, BOARD pin 12   
landing_tag={}
def gpio_controll():
    # Pin Setup:
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
    print("Starting demo now! Press CTRL+C to exit")
    curr_value = GPIO.HIGH
    GPIO.output(output_pin, curr_value)
    time.sleep(1)
    GPIO.output(output_pin,GPIO.LOW)
    

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

def tag_detections_cb(msg):
    global detected_tags
    detected_tags.clear()
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
    for det in msg.detections:
        tag_id = det.id[0]
        pos = det.pose.pose.pose.position
        landing_tag[tag_id] = (pos.x, pos.y, pos.z)

def adjust_pose_for_landing(pose,offset):
    dx,dy,dz = offset
    pose.pose.position.x -= dx
    pose.pose.position.y += dy
    pose.pose.position.z -= dz+0.05
    return pose

if __name__ == "__main__":
    rospy.init_node("offboard_apriltag_node")
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    current_pose_sub = rospy.Subscriber("mavros/vision_pose/pose", PoseStamped, callback=pose_cb)
    tag_sub = rospy.Subscriber("/d455/tag_detections", AprilTagDetectionArray, callback=tag_detections_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    tag_sub_usb = rospy.Subscriber("/usb_cam/tag_detections",AprilTagDetectionArray,landing_tag_callback)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    # 初始化航点列表和每个航点对应tag_id
    waypoints = [
        {"pos": (0,-0.75, 1), "tag_id": 1},
        {"pos": (0,-0.75, 1.4), "tag_id": 2},
        {"pos": (0,-1.25, 1.4), "tag_id": 3},
        {"pos": (0,-1.25, 1.0), "tag_id": 4},
        {"pos": (0,-1.75, 1.0), "tag_id": 5},
        {"pos": (0,-1.75, 1.4), "tag_id": 6},
    ]

    pose = PoseStamped()
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    # 模式切换与解锁
    offb_set_mode = SetModeRequest(custom_mode='OFFBOARD')
    arm_cmd = CommandBoolRequest(value=True)
    last_req = rospy.Time.now()

    wp_index = 0
    reached_wp = False

    while not rospy.is_shutdown():
        # 模式切换与解锁处理
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5):
            set_mode_client.call(offb_set_mode)
            last_req = rospy.Time.now()
        elif not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5):
            arming_client.call(arm_cmd)
            last_req = rospy.Time.now()

        # 航点处理
        if wp_index < len(waypoints):
            wp = waypoints[wp_index]
            pose.pose.position.x = wp["pos"][0]
            pose.pose.position.y = wp["pos"][1]
            pose.pose.position.z = wp["pos"][2]

            local_pos_pub.publish(pose)

            dx = current_pose.pose.position.x - pose.pose.position.x
            dy = current_pose.pose.position.y - pose.pose.position.y
            dz = current_pose.pose.position.z - pose.pose.position.z
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

            if dist < 0.3 and not reached_wp:
                rospy.loginfo(f"到达航点 {wp_index + 1}，等待 AprilTag {wp['tag_id']} 检测")

                wait_time = rospy.Time.now()
                found = False
                while (rospy.Time.now() - wait_time).to_sec() < 5.0:
                    if wp["tag_id"] in detected_tags:
                        offset = detected_tags[wp["tag_id"]]
                        rospy.loginfo(f"检测到 tag {wp['tag_id']}，偏移: {offset}")
                        adjust_pose = adjust_pose_for_apriltag(pose, offset)
                        for i in range(40):
                            local_pos_pub.publish(adjust_pose)
                            rate.sleep()
                        found = True
                        gpio_controll()
                        break
                    rate.sleep()

                if not found:
                    rospy.logwarn(f"未检测到 tag {wp['tag_id']}，跳过微调")

                wp_index += 1
                reached_wp = True
            elif dist >= 0.3:
                reached_wp = False

        else:
            
            rospy.loginfo("所有航点完成,准备返航")
            pose.pose.position.x=0
            pose.pose.position.y=0
            pose.pose.position.z=0.3
            for i in range(40):
                local_pos_pub.publish(pose)
                rate.sleep()
            if math.sqrt(current_pose.pose.position.x**2+current_pose.pose.position.y**2) <= 0.2:
                if 0 in landing_tag:
                    rospy.logwarn(f"检测到降落tag,准备降落!")
                    offset = landing_tag[0]
                    adjusted_pose=adjust_pose_for_landing(current_pose,offset)
                    for i in range(40):
                        local_pos_pub.publish(adjusted_pose)
                        rate.sleep()  
                    land_set_mode = SetModeRequest(custom_mode="AUTO.LAND")
                    set_mode_client.call(land_set_mode)
                else:
                    rospy.logwarn(f"没有检测到tag，在当前位置降落")
                    land_set_mode = SetModeRequest(custom_mode="AUTO.LAND")
                    set_mode_client.call(land_set_mode)
                    GPIO.cleanup()
                break
        
        rate.sleep()
