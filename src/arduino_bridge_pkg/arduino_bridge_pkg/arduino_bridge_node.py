#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import serial
from std_msgs.msg import Int32


class ArduinoBridgeNode(Node):
    def __init__(self):
        # 노드 이름: arduino_bridge_node
        super().__init__('arduino_bridge_node')

        port = '/dev/ttyACM0'   # 환경에 따라 /dev/ttyUSB0 일 수도 있음
        baud = 115200

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1)
            self.get_logger().info(f"Serial port opened: {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial: {e}")
            self.ser = None

        # lane_follow_node의 조향각 토픽 구독
        self.sub = self.create_subscription(
            Int32,
            'lane_follow/steering_angle',
            self.steering_callback,
            10
        )

    def steering_callback(self, msg: Int32):
        if self.ser is None:
            return

        angle = msg.data
        data = f"{angle}\n"

        try:
            self.ser.write(data.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()