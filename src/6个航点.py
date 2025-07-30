#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
current_state = State()
current_pose = PoseStamped()
arm_cmd = CommandBoolRequest()
# 初始化航点列表
waypoints = [
    {"pos": (0, 0.75, 1), "tag_id": 1},
    {"pos": (0, 0.75, 1.4), "tag_id": 2},
    {"pos": (0, 0.25, 1.4), "tag_id": 3},
    {"pos": (0, 1.25, 1.0), "tag_id": 4},
    {"pos": (0, 1.75, 1.0), "tag_id": 5},
    {"pos": (0, 1.75, 1.4), "tag_id": 6},
    {"pos": (0, 0, 0.3), "tag_id": 0},
]

def state_cb(msg):
    global current_state
    current_state = msg
   


def pose_cb(msg):
    global current_pose
    current_pose = msg
   

if __name__ == "__main__":
    rospy.init_node("waypoint_navigation_node")

    
    state_sub = rospy.Subscriber("iris_0/mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("iris_0/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("iris_0/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("iris_0/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("iris_0/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("iris_0/mavros/set_mode", SetMode)
    rate = rospy.Rate(20)
    pose_sub = rospy.Subscriber("iris_0/mavros/local_position/pose", PoseStamped, callback=pose_cb)
    # 等待连接
    while not rospy.is_shutdown() and not current_state.connected:
        print("等待无人机连接...")
        rate.sleep()


    pose = PoseStamped()

    # 发布初始位置
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    # 设置飞行模式为 OFFBOARD
    offb_set_mode = SetModeRequest(custom_mode='OFFBOARD')
    
    arm_cmd.value = True
    last_req = rospy.Time.now()
    wp_index = 0
    reached_wp = False


    while not rospy.is_shutdown():
        # 模式切换处理
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5):
            set_mode_client.call(offb_set_mode)
            last_req = rospy.Time.now()
            rospy.loginfo("OFFBORAD enabled")
        if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

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
            print(dist)
            if dist < 0.3 and not reached_wp:
                rospy.loginfo(f"到达航点 {wp_index + 1}")
                wp_index += 1
                reached_wp = True
            elif dist >= 0.3:
                reached_wp = False
        else:
            rospy.loginfo("所有航点完成")
            land_set_mode = SetModeRequest(custom_mode="AUTO.LAND")
            set_mode_client.call(land_set_mode)
            rospy.loginfo("进入降落模式")
            break

        rate.sleep()
