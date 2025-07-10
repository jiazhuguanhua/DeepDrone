#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
import math
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg



current_x, current_y, current_z = None, None, None  # 初始化为 None
def odom_callback(msg):
        # 更新当前位置
        global current_x, current_y, current_z
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = msg.pose.pose.position.z



# 定义全局变量用于存储 PID 控制器的状态
prev_error_x = 0.0
prev_error_y = 0.0
prev_error_z = 0.0
integral_x = 0.0
integral_y = 0.0
integral_z = 0.0

def compute_velocity(pose):
    global current_x, current_y, current_z
    global prev_error_x, prev_error_y, prev_error_z
    global integral_x, integral_y, integral_z

    if current_x is None or current_y is None or current_z is None:
        rospy.logwarn("Waiting for odometry data...")
        rospy.loginfo(f"Current position: x={current_x}, y={current_y}, z={current_z}")
        return None

    vel_msg = TwistStamped()  # 使用 TwistStamped

    # PID 参数
    kp = 2  # 比例系数
    ki = 0.1  # 积分系数
    kd = 0.05  # 微分系数

    # 计算误差
    error_x = pose.pose.position.x - current_x
    error_y = pose.pose.position.y - current_y
    error_z = pose.pose.position.z - current_z

    # 计算积分项
    integral_x += error_x
    integral_y += error_y
    integral_z += error_z

    # 计算微分项
    derivative_x = error_x - prev_error_x
    derivative_y = error_y - prev_error_y
    derivative_z = error_z - prev_error_z

    # 更新前一次误差
    prev_error_x = error_x
    prev_error_y = error_y
    prev_error_z = error_z


    # 计算 PID 输出
    vel_msg.twist.linear.x = kp * error_x + ki * integral_x + kd * derivative_x
    vel_msg.twist.linear.y = kp * error_y + ki * integral_y + kd * derivative_y
    vel_msg.twist.linear.z = kp * error_z + ki * integral_z + kd * derivative_z
 
    # 限制最大速度
    max_speed =2
    vel_msg.twist.linear.x = max(min(vel_msg.twist.linear.x, max_speed), -max_speed)
    vel_msg.twist.linear.y = max(min(vel_msg.twist.linear.y, max_speed), -max_speed)
    vel_msg.twist.linear.z = max(min(vel_msg.twist.linear.z, max_speed), -max_speed)
 
    # 添加时间戳
    vel_msg.header.stamp = rospy.Time.now()

    return vel_msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("iris_0/mavros/state", State, callback = state_cb)
        
    vel_pub = rospy.Publisher('iris_0/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)# 发布速度指令

    odom_sub = rospy.Subscriber('iris_0/mavros/local_position/odom', Odometry, odom_callback) # 订阅当前位置
   
    #local_pos_pub = rospy.Publisher("iris_0/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("iris_0/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("iris_0/mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("iris_0/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("iris_0/mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        vel_msg=compute_velocity(pose)
        
        if vel_msg:  # 确保速度指令有效
            vel_pub.publish(vel_msg)
           
        rate.sleep()
