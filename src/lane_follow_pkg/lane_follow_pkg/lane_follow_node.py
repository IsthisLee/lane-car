#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32


class LaneFollowNode(Node):
    def __init__(self):
        # 노드 이름 고정: lane_follow_node
        super().__init__('lane_follow_node')

        self.bridge = CvBridge()

        # camera_node가 퍼블리시하는 토픽 구독
        self.sub_img = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # 조향각(0~180도) 발행
        self.pub_steer = self.create_publisher(
            Int32,
            'lane_follow/steering_angle',
            10
        )

        # PD 계수 (튜닝 대상)
        self.kp = 0.15
        self.kd = 0.4
        self.prev_error = 0.0

        # 차선 중심 계산할 y 위치
        self.y_ref = 400

        # 마지막 왼/오른쪽 차선 정보 저장 (차선 끊겼을 때 사용 가능)
        self.last_left_fit = None    # (slope, intercept)
        self.last_right_fit = None

    def image_callback(self, msg: Image):
        # 1) ROS Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape

        # 2) Gray → Threshold → Canny
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(binary, 80, 200)

        # 3) 사다리꼴 ROI 마스크
        polygon = np.array([[
            (0, height),
            (width, height),
            (int(width * 0.6), int(height * 0.5)),
            (int(width * 0.4), int(height * 0.5))
        ]], np.int32)
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, polygon, 255)
        roi_edges = cv2.bitwise_and(edges, mask)

        # 4) 허프 변환으로 선분 후보 검출
        lines = cv2.HoughLinesP(
            roi_edges,
            1,
            np.pi / 180,
            threshold=50,
            minLineLength=50,
            maxLineGap=30
        )

        left_fit, right_fit = self.get_lane_lines(lines)

        # 마지막 차선 정보 업데이트
        if left_fit is not None:
            self.last_left_fit = left_fit
        if right_fit is not None:
            self.last_right_fit = right_fit

        # 둘 중 하나라도 None이면, 이전 프레임 사용 (선택)
        if left_fit is None:
            left_fit = self.last_left_fit
        if right_fit is None:
            right_fit = self.last_right_fit

        steering_angle = 90
        error = 0.0

        if left_fit is not None and right_fit is not None:
            # y = m x + b → x = (y - b) / m
            x_left = (self.y_ref - left_fit[1]) / left_fit[0]
            x_right = (self.y_ref - right_fit[1]) / right_fit[0]

            lane_center = (x_left + x_right) / 2.0
            image_center = width / 2.0

            error = lane_center - image_center

            # PD 제어
            derivative = error - self.prev_error
            steering_angle = 90 - int(self.kp * error + self.kd * derivative)
            self.prev_error = error

        # 0~180 범위 제한
        steering_angle = max(0, min(180, steering_angle))

        # 5) 토픽 발행
        msg_out = Int32()
        msg_out.data = int(steering_angle)
        self.pub_steer.publish(msg_out)

        # 디버그 시각화 (선택)
        debug = frame.copy()
        cv2.polylines(debug, polygon, True, (0, 255, 0), 2)

        if left_fit is not None:
            self.draw_line(debug, left_fit, height, (0, 255, 255))
        if right_fit is not None:
            self.draw_line(debug, right_fit, height, (0, 255, 255))

        image_center = int(width / 2)
        cv2.line(debug, (image_center, 0), (image_center, height), (255, 0, 0), 2)

        cv2.putText(debug, f"angle: {steering_angle}", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.putText(debug, f"error: {error:.1f}", (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        cv2.imshow("lane_follow_debug", debug)
        cv2.waitKey(1)

    def get_lane_lines(self, lines):
        if lines is None:
            return None, None

        left_lines = []
        right_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1

            if slope < 0:
                left_lines.append((slope, intercept))
            else:
                right_lines.append((slope, intercept))

        left_fit = np.mean(left_lines, axis=0) if left_lines else None
        right_fit = np.mean(right_lines, axis=0) if right_lines else None

        return left_fit, right_fit

    def draw_line(self, image, line_fit, height, color):
        slope, intercept = line_fit
        y1 = height
        y2 = int(height * 0.5)
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        cv2.line(image, (x1, y1), (x2, y2), color, 5)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
