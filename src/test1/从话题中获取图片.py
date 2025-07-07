#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import RPi.GPIO as GPIO
import time
import threading

# GPIO引脚定义
OUTPUT_PIN = 18  # BCM 18 (Board 12)

class QRCodeScanner:
    def __init__(self):
        rospy.init_node('qr_code_scanner', anonymous=True)
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        # 初始化GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(OUTPUT_PIN, GPIO.OUT, initial=GPIO.LOW)

        # 保存已识别二维码内容
        self.detected_qrcodes = set()
        self.lock = threading.Lock()  # 用于保护set的线程安全

        # 订阅图像话题
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.loginfo("二维码识别节点已启动。")
        rospy.on_shutdown(self.cleanup)
        rospy.spin()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("图像转换失败: %s", str(e))
            return

    # 获取图像中心
        height, width, _ = frame.shape
        center_x_img = width // 2
        center_y_img = height // 2
        center_margin_x = width * 0.3  # 中心区域宽度 ±30%
        center_margin_y = height * 0.3

    # 检测二维码
        data, bbox, _ = self.qr_detector.detectAndDecode(frame)

        if data and bbox is not None:
        # 计算二维码中心点
            cx = int(bbox[0][:, 0].mean())
            cy = int(bbox[0][:, 1].mean())

        # 判断二维码是否靠近中心
            in_center = (
                abs(cx - center_x_img) <= center_margin_x and
                abs(cy - center_y_img) <= center_margin_y
                )

            if in_center:
                with self.lock:
                    if data not in self.detected_qrcodes:
                        rospy.loginfo("新二维码 %s 出现在图像中心，触发LED", data)
                        self.detected_qrcodes.add(data)
                        threading.Thread(target=self.trigger_led).start()

            else:
                rospy.loginfo("二维码 %s 在边缘区域，忽略触发", data)

        # 可视化边框
            bbox = bbox.astype(int)
            for i in range(len(bbox[0])):
                pt1 = tuple(bbox[0][i])
                pt2 = tuple(bbox[0][(i + 1) % len(bbox[0])])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
        # 标出二维码中心
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

    # 显示图像
        cv2.imshow("QR Code Detection", frame)
        cv2.waitKey(1)

    def trigger_led(self):
        """点亮LED两秒"""
        GPIO.output(OUTPUT_PIN, GPIO.HIGH)
        time.sleep(2)
        GPIO.output(OUTPUT_PIN, GPIO.LOW)

    def cleanup(self):
        """关闭前的清理"""
        rospy.loginfo("清理 GPIO 并退出。")
        GPIO.output(OUTPUT_PIN, GPIO.LOW)
        GPIO.cleanup()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        QRCodeScanner()
    except rospy.ROSInterruptException:
        pass
