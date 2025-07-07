#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from mavros_msgs.srv import SetMode

import tf2_ros
import tf2_geometry_msgs

class PrecisionLandingNode:
    def __init__(self):
        rospy.init_node("precision_landing_node")

        self.current_pose_odom = None
        self.tag_detected = False
        self.last_tag_time = rospy.Time(0)
        self.tag_timeout = rospy.Duration(1.0)  # 1秒内未检测到Tag则悬停

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Assume the world frame published by VINS or MAVROS local position is 'd455_camera_init'
        # Assume the drone body frame is 'base_footprint'
        # Assume the USB camera frame is 'usb_cam'
        self.world_frame_id = "d455_camera_init" # Your defined world frame
        self.drone_body_frame_id = "base_footprint" # Your drone body frame
        self.camera_frame_id = "usb_cam" # Your USB camera frame

        # Subscribers
        # Using Odometry for drone's pose in world frame (adjust topic if needed)
        rospy.Subscriber("/vins_fusion/imu_propagate", Odometry, self.odom_callback)
        # TODO: Change topic to the actual USB camera apriltag topic
        rospy.Subscriber("/apriltag_bottom/tag_detections", AprilTagDetectionArray, self.tag_callback) # Subscribing to a specific topic

        # Publishers
        self.setpoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Service 客户端：切换模式
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)  # 10Hz

        rospy.loginfo("Precision landing node with safety started.")
        # rospy.spin() # Don't call spin here if using a timer

    def odom_callback(self, msg):
        # Store the full Odometry message to get header (for timestamp and frame_id)
        self.current_pose_odom = msg

    def tag_callback(self, msg):
        if len(msg.detections) > 0:
            # Assume we care about the first detected tag
            self.tag_detected = True
            self.last_tag_time = rospy.Time.now()

            # Get the pose of the tag relative to the camera frame
            # The pose in the message is relative to the frame_id specified in the header of DetectionArray
            # which should be your camera frame ('usb_cam').
            tag_pose_in_cam_frame = msg.detections[0].pose.pose.pose

            # Store the tag pose and the timestamp of the detection
            self.tag_pose_in_cam_frame = tag_pose_in_cam_frame
            self.tag_detection_timestamp = msg.header.stamp

        else:
            self.tag_detected = False
            # Clear the stored tag pose when no tags are detected
            self.tag_pose_in_cam_frame = None

    def control_loop(self, event):
        if self.current_pose_odom is None:
            rospy.logwarn_throttle(5, "Waiting for drone position (odom)...")
            return

        if not self.tag_detected or rospy.Time.now() - self.last_tag_time > self.tag_timeout:
            rospy.logwarn_throttle(1, "AprilTag lost or timed out. Holding position.")
            # Optional: Publish the current drone position as the setpoint to explicitly hold
            # If you don't publish, MAVROS might hold the last commanded position
            # You need to confirm MAVROS behavior when setpoint messages stop.
            # A simple way to hold is to publish the current estimated position.
            hold_pose = PoseStamped()
            hold_pose.header.stamp = rospy.Time.now()
            hold_pose.header.frame_id = self.world_frame_id # Assuming drone pose is in world frame
            hold_pose.pose = self.current_pose_odom.pose.pose # Use latest known drone pose
            self.setpoint_pub.publish(hold_pose)
            return

        # --- TF Transformations ---
        # We have tag_pose_in_cam_frame (AprilTag relative to usb_cam)
        # We need to transform it to world frame (d455_camera_init)

        # 1. Create a PoseStamped for the AprilTag position in the camera frame
        # We use PointStamped because we only need the position for alignment
        tag_point_in_cam_frame = PointStamped()
        tag_point_in_cam_frame.header.stamp = self.tag_detection_timestamp # Use the timestamp of the tag detection
        tag_point_in_cam_frame.header.frame_id = self.camera_frame_id # Frame is 'usb_cam'
        tag_point_in_cam_frame.point = self.tag_pose_in_cam_frame.position # Get the position part

        # 2. Transform the Tag position from camera frame ('usb_cam') to the world frame ('d455_camera_init')
        # We need to wait for the transform to become available.
        # The required transforms are: usb_cam -> base_footprint (static) and base_footprint -> d455_camera_init (from drone's pose)
        # TF can chain these automatically. We ask for transform directly from usb_cam to world frame.
        try:
            # Look up the transform from camera_frame_id to world_frame_id
            # at the timestamp of the tag detection
            transform_cam_to_world = self.tf_buffer.lookup_transform(
                self.world_frame_id, # Target frame
                tag_point_in_cam_frame.header.frame_id, # Source frame (usb_cam)
                tag_point_in_cam_frame.header.stamp, # Lookup time
                rospy.Duration(0.1) # Timeout duration
            )

            # Apply the transformation to the tag point
            tag_point_in_world_frame = self.tf_buffer.transform(tag_point_in_cam_frame, self.world_frame_id)

            # The Tag position in the world frame (d455_camera_init) is now tag_point_in_world_frame.point

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logerr_throttle(1, "Could not transform tag pose: %s" % ex)
            return # Cannot proceed without valid transformation

        # --- Calculate Target Setpoint ---

        # The AprilTag position in the world frame
        tag_world_x = tag_point_in_world_frame.point.x
        tag_world_y = tag_point_in_world_frame.point.y
        # tag_world_z is the height of the tag in the world frame. Assuming tag is on the ground (Z=0 in world frame)
        # If tag height is not 0, you need to adjust or use the full transformed pose.
        # For landing *on* the tag, the target horizontal position should be the tag's horizontal position.
        # The target Z should be the desired landing height relative to the world frame origin (or ground level).

        # Current drone height (in world frame) - using the latest available odom pose
        drone_z = self.current_pose_odom.pose.pose.position.z

        # 如果高度很低，自动进入 LAND 模式 (assuming 0.3m is relative to world Z=0)
        if drone_z <= 0.3:
            try:
                res = self.set_mode_srv(0, "AUTO.LAND")
                if res.mode_sent:
                    rospy.loginfo("Switched to AUTO.LAND mode.")
                else:
                    rospy.logwarn("Failed to switch to AUTO.LAND.")
            except rospy.ServiceException as e:
                rospy.logerr("Set mode service call failed: %s" % str(e))
            return # Stop sending setpoints after switching mode

        # Normal descending - send a target position slightly below the current drone height, aligned horizontally with the tag
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = self.world_frame_id # Setpoint is in world frame

        # Target horizontal position is the detected tag's horizontal position in the world frame
        target_pose.pose.position.x = tag_world_x
        target_pose.pose.position.y = tag_world_y

        # Target vertical position (descend gradually)
        # Descend 10cm each step, but not below a safety height (e.g., 0.2m relative to world Z=0)
        target_pose.pose.position.z = max(drone_z - 0.1, 0.2) # Descend 10cm, minimum height 0.2m

        # Maintain current yaw or set a fixed yaw (e.g., aligned with world X)
        # For precise landing, maintaining consistent orientation might be good.
        # Simplest is identity quaternion (no rotation change relative to world frame orientation)
        target_pose.pose.orientation.w = 1.0
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0

        self.setpoint_pub.publish(target_pose)

        rospy.loginfo_throttle(1, "Tag in world: x=%.2f y=%.2f (target Z=%.2f). Current Drone Z: %.2f" %
                               (tag_world_x, tag_world_y, target_pose.pose.position.z, drone_z))
        # print more detailed debug info if needed
        # rospy.loginfo_throttle(5, f"Tag in Cam: x={self.tag_pose_in_cam_frame.position.x:.2f} y={self.tag_pose_in_cam_frame.position.y:.2f} z={self.tag_pose_in_cam_frame.position.z:.2f}")
        # rospy.loginfo_throttle(5, f"Tag World Transformed: x={tag_point_in_world_frame.point.x:.2f} y={tag_point_in_world_frame.point.y:.2f} z={tag_point_in_world_frame.point.z:.2f}")

if __name__ == '__main__':
    try:
        node = PrecisionLandingNode()
        rospy.spin() # Now call spin here to keep the node alive for subscribers and timer callbacks
    except rospy.ROSInterruptException:
        pass

