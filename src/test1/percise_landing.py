#!/usr/bin/env python3
import rospy, sys, tf2_ros, math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import LandingTarget
from mavros_msgs.srv import CommandLong, SetMode

# ---------- 参数 ----------
#vehicle_id = sys.argv[1] if len(sys.argv) > 1 else "0"
topic_prefix = "/mavros"          # 单机可直接用 /mavros
frame_id   = "tag_" + "0"  # AprilTag TF frame

# ---------- 初始化 ----------
rospy.init_node("px4_precision_landing")
tf_buf   = tf2_ros.Buffer()
tf_lis   = tf2_ros.TransformListener(tf_buf)
lt_pub   = rospy.Publisher(topic_prefix + "/landing_target/raw", LandingTarget, queue_size=1)
set_mode = rospy.ServiceProxy(topic_prefix + "/cmd/set_mode", SetMode)
rate = rospy.Rate(30)

# ---------- 主循环 ----------
while not rospy.is_shutdown():
    try:
        tf = tf_buf.lookup_transform("map", frame_id, rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
        continue

    # 1. 构造官方 LandingTarget 消息
    msg = LandingTarget()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.angle_valid = True
    msg.distance_valid = True
    msg.size_valid   = False

    # 计算水平角度（rad）与斜距
    dx = tf.transform.translation.x
    dy = tf.transform.translation.y
    dz = tf.transform.translation.z
    msg.angle_x = math.atan2(dx, dz)
    msg.angle_y = math.atan2(dy, dz)
    msg.distance = math.sqrt(dx*dx + dy*dy + dz*dz)

    lt_pub.publish(msg)

    # 2. 检测到 tag 后切 LAND 模式（只切一次）
    if not hasattr(rospy, "_land_sent"):
        try:
            set_mode(custom_mode="LAND")
            rospy.loginfo("Switch to LAND mode")
            rospy._land_sent = True
        except rospy.ServiceException as e:
            rospy.logwarn("SetMode service call failed: %s", e)

    rate.sleep()