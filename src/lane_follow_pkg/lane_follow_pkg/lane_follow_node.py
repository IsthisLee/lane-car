#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import collections


def average_slope_intercept(image, lines):
    left_fit, right_fit = [], []
    if lines is None:
        return None, None

    width = image.shape[1]
    center_x = width / 2

    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope, intercept = parameters

        if slope < -0.5 and x1 < center_x and x2 < center_x:
            left_fit.append((slope, intercept))
        elif slope > 0.5 and x1 > center_x and x2 > center_x:
            right_fit.append((slope, intercept))

    left_fit_avg = np.average(left_fit, axis=0) if left_fit else None
    right_fit_avg = np.average(right_fit, axis=0) if right_fit else None
    return left_fit_avg, right_fit_avg


def make_coordinates(image, line_parameters):
    if line_parameters is None:
        return None
    slope, intercept = line_parameters
    height = image.shape[0]
    y1 = height
    y2 = int(height * 0.7)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])


class LaneFollowNode(Node):
    def __init__(self):
        super().__init__('lane_follow_node')
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.pub_steer = self.create_publisher(
            Int32, 'lane_follow/steering_angle', 10)

        self.recent_angles = collections.deque(maxlen=5)
        self.last_angle = 90

        self.last_left_fit = None
        self.last_right_fit = None

        self.kp = -0.5
        self.kd = -0.15
        self.prev_error = 0
        self.LANE_WIDTH_PIXELS = 350
        self.y_ref = 0

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        blurred = cv2.GaussianBlur(binary, (5, 5), 0)
        edges = cv2.Canny(blurred, 80, 200)

        height, width = edges.shape
        self.y_ref = int(height * 0.6)

        mask = np.zeros_like(edges)

        polygon = np.array([[
            (0, height),
            (width, height),
            (int(width * 0.7), int(height / 3)),
            (int(width * 0.3), int(height / 3))
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        roi_edges = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(roi_edges, 1, np.pi / 180, 50,
                                minLineLength=50, maxLineGap=10)

        left_avg, right_avg = average_slope_intercept(cv_image, lines)

        if left_avg is not None:
            self.last_left_fit = left_avg
        if right_avg is not None:
            self.last_right_fit = right_avg

        line_image = np.zeros_like(cv_image)
        steering_angle = 90
        image_center = width / 2
        error = 0

        lane_center = None

        if self.last_left_fit is not None and self.last_right_fit is not None:
            x_left = (self.y_ref - self.last_left_fit[1]) / self.last_left_fit[0]
            x_right = (self.y_ref - self.last_right_fit[1]) / self.last_right_fit[0]
            lane_center = (x_left + x_right) / 2
            error = lane_center - image_center
        elif self.last_left_fit is not None:
            x_left = (self.y_ref - self.last_left_fit[1]) / self.last_left_fit[0]
            lane_center = x_left + self.LANE_WIDTH_PIXELS / 2
            error = lane_center - image_center
        elif self.last_right_fit is not None:
            x_right = (self.y_ref - self.last_right_fit[1]) / self.last_right_fit[0]
            lane_center = x_right - self.LANE_WIDTH_PIXELS / 2
            error = lane_center - image_center

        if lane_center is not None:
            derivative = error - self.prev_error
            steering_angle = 90 - int(self.kp * error + self.kd * derivative)
            self.prev_error = error
        else:
            steering_angle = self.last_angle

        steering_angle = max(0, min(180, steering_angle))
        self.recent_angles.append(steering_angle)
        smoothed_angle = int(sum(self.recent_angles) / len(self.recent_angles))
        self.last_angle = smoothed_angle

        msg_out = Int32()
        msg_out.data = smoothed_angle
        self.pub_steer.publish(msg_out)
        self.get_logger().info(f'Publishing Angle: {smoothed_angle}, Error: {error:.2f}')

        left_coords = make_coordinates(cv_image, self.last_left_fit)
        right_coords = make_coordinates(cv_image, self.last_right_fit)

        if left_coords is not None:
            cv2.line(line_image, (left_coords[0], left_coords[1]), (left_coords[2], left_coords[3]), (0, 0, 255), 10)
        if right_coords is not None:
            cv2.line(line_image, (right_coords[0], right_coords[1]), (right_coords[2], right_coords[3]), (255, 0, 0), 10)

        overlay = np.zeros_like(cv_image)
        cv2.fillPoly(overlay, polygon, (0, 255, 255))
        combo = cv2.addWeighted(cv_image, 0.8, overlay, 0.3, 1)

        combo = cv2.addWeighted(combo, 0.8, line_image, 1, 1)
        cv2.line(combo, (0, self.y_ref), (width, self.y_ref), (0, 255, 255), 1)
        cv2.circle(combo, (int(image_center), self.y_ref), 5, (0, 255, 0), -1)

        if lane_center is not None:
            cv2.circle(combo, (int(lane_center), self.y_ref), 5, (255, 0, 255), -1)

        cv2.imshow('Lane Detection', combo)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
