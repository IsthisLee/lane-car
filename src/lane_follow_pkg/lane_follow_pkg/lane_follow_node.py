# lane_follow_pkg/lane_follow_pkg/lane_follow_node.py

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
        super().__init__('lane_follow_node')

        self.bridge = CvBridge()

        self.sub_img = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.pub_steer = self.create_publisher(
            Int32,
            'lane_follow/steering_angle',
            10
        )

        # PD 계수 (필요시 튜닝)
        self.kp = 0.15
        self.kd = 0.4
        self.prev_error = 0.0

        self.y_ref = 400  # 차선 중심 계산할 y 위치

        self.last_left_fit = None   # (slope, intercept)
        self.last_right_fit = None

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # BGR, 컬러
        height, width, _ = frame.shape

        # 1) Gray → Threshold → Canny
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY) # 이진화
        edges = cv2.Canny(binary, 80, 200)

        # 2) 사다리꼴 ROI
        polygon = np.array([[
            (int(width * 0.05), height),
            (int(width * 0.95), height),
            (int(width * 0.78), int(height * 0.38)),
            (int(width * 0.22), int(height * 0.38))
        ]], np.int32)

        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, polygon, 255)
        roi_edges = cv2.bitwise_and(edges, mask)

        # 3) 허프 변환 (직선 검출)
        lines = cv2.HoughLinesP(
            roi_edges,
            1,
            np.pi / 180,
            threshold=50,
            minLineLength=50,
            maxLineGap=30
        )

        left_fit, right_fit = self.get_lane_lines(lines)

        # 마지막 유효 차선 저장
        if left_fit is not None:
            self.last_left_fit = left_fit
        if right_fit is not None:
            self.last_right_fit = right_fit

        if left_fit is None:
            left_fit = self.last_left_fit
        if right_fit is None:
            right_fit = self.last_right_fit

        steering_angle = 90
        error = 0.0

        if left_fit is not None and right_fit is not None:
            mL, bL = left_fit
            mR, bR = right_fit

            # slope/offset 유효성 검사
            if (abs(mL) >= 1e-3 and abs(mR) >= 1e-3 and
                    np.isfinite(mL) and np.isfinite(bL) and
                    np.isfinite(mR) and np.isfinite(bR)):

                x_left = (self.y_ref - bL) / mL
                x_right = (self.y_ref - bR) / mR

                if np.isfinite(x_left) and np.isfinite(x_right):
                    lane_center = (x_left + x_right) / 2.0
                    image_center = width / 2.0

                    error = lane_center - image_center
                    derivative = error - self.prev_error
                    steering_angle = 90 - int(self.kp * error + self.kd * derivative)
                    self.prev_error = error
            # 그 외 경우는 steering_angle = 90 유지

        steering_angle = max(0, min(180, steering_angle))

        msg_out = Int32()
        msg_out.data = int(steering_angle)
        self.pub_steer.publish(msg_out)

        # 디버그 시각화
        debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
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
        cv2.imshow("binary", binary)
        cv2.imshow("edges", edges)
        cv2.imshow("roi_edges", roi_edges)

        cv2.waitKey(1)

    def get_lane_lines(self, lines):
        if lines is None:
            return None, None

        left_lines = []
        right_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            dy = y2 - y1
            if dx == 0:
                continue  # 수직선은 스킵

            slope = dy / dx
            # 너무 수평에 가까운 선 제거
            if abs(slope) < 0.1:
                continue

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

        if slope == 0 or not np.isfinite(slope) or not np.isfinite(intercept):
            return

        y1 = height
        y2 = int(height * 0.5)

        x1 = (y1 - intercept) / slope
        x2 = (y2 - intercept) / slope

        if not np.isfinite(x1) or not np.isfinite(x2):
            return

        x1 = int(round(x1))
        x2 = int(round(x2))

        cv2.line(image, (x1, y1), (x2, y2), color, 5)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
