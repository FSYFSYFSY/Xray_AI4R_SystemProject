#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from ai4r_interfaces.msg import EscAndSteeringPercent


class TestSteeringCommander(Node):
    def __init__(self) -> None:
        super().__init__('test_steering_commander')
        self.publisher_ = self.create_publisher(EscAndSteeringPercent, 'esc_and_steering_set_point_percent_action', 10)
        self.period_seconds = 4.0
        self.timer_period_seconds = 0.02  # 50 Hz updates for a smooth waveform
        self.timer = self.create_timer(self.timer_period_seconds, self._on_timer)
        self.esc_cmd = 0.0
        self.esc_min = -30.0
        self.esc_max = 30.0
        self.esc_mid = (self.esc_min + self.esc_max) / 2.0
        self.esc_amplitude = (self.esc_max - self.esc_min) / 2.0
        self.steer_min = -60.0
        self.steer_max = 60.0
        self.steer_mid = (self.steer_min + self.steer_max) / 2.0
        self.steer_amplitude = (self.steer_max - self.steer_min) / 2.0
        self.start_time_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f'Test commander started')
        self.get_logger().info(f'ESC      sinusoid {self.esc_min} to {self.esc_min} percent with {self.period_seconds} second period')
        self.get_logger().info(f'Steering sinusoid {self.steer_min} to {self.steer_min} percent with {self.period_seconds} second period')

    def _on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        elapsed_seconds = (now_ns - self.start_time_ns) * 1e-9
        angle = (2.0 * math.pi * elapsed_seconds) / self.period_seconds

        esc_cmd = self.esc_mid + self.esc_amplitude * math.sin(angle)

        steering_cmd = self.steer_mid + self.steer_amplitude * math.sin(angle)

        message = EscAndSteeringPercent()
        message.esc_percent = float(esc_cmd)
        message.steering_percent = float(steering_cmd)
        self.publisher_.publish(message)


def main() -> None:
    rclpy.init()
    node = TestSteeringCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()