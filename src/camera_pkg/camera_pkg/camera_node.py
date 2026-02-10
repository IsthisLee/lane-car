#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # cv2.VideoCapture로 웹캠 열기
        self.cap = cv2.VideoCapture(4)
        if not self.cap.isOpened():
            self.get_logger().error("웹캠을 열 수 없습니다!")
            rclpy.shutdown()
            return

        # 30 FPS 타이머
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("프레임을 읽지 못했습니다.")
            return

        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
