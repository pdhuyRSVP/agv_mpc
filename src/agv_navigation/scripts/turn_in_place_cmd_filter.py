#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurnInPlaceCmdFilter(Node):
    def __init__(self):
        super().__init__('turn_in_place_cmd_filter')
        self.declare_parameter('input_topic', '/cmd_vel_nav_smooth')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('angular_enter_threshold', 0.18)
        self.declare_parameter('angular_exit_threshold', 0.06)
        self.declare_parameter('hard_stop_angular_threshold', 0.38)
        self.declare_parameter('min_forward_to_keep_turning', 0.04)
        self.declare_parameter('zero_y_when_turning', True)
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('reverse_max_abs', 0.08)
        self.declare_parameter('reverse_block_angular_z', 0.20)
        self.declare_parameter('max_forward_velocity', 0.40)
        self.declare_parameter('max_angular_velocity', 0.40)
        self.declare_parameter('corner_slowdown_start', 0.10)
        self.declare_parameter('corner_full_slowdown_angular', 0.30)
        self.declare_parameter('corner_max_forward_velocity', 0.20)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.angular_enter_threshold = float(
            self.get_parameter('angular_enter_threshold').value)
        self.angular_exit_threshold = float(
            self.get_parameter('angular_exit_threshold').value)
        self.hard_stop_angular_threshold = float(
            self.get_parameter('hard_stop_angular_threshold').value)
        self.min_forward_to_keep_turning = float(
            self.get_parameter('min_forward_to_keep_turning').value)
        self.zero_y_when_turning = bool(self.get_parameter('zero_y_when_turning').value)
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.reverse_max_abs = float(self.get_parameter('reverse_max_abs').value)
        self.reverse_block_angular_z = float(
            self.get_parameter('reverse_block_angular_z').value)
        self.max_forward_velocity = float(
            self.get_parameter('max_forward_velocity').value)
        self.max_angular_velocity = float(
            self.get_parameter('max_angular_velocity').value)
        self.corner_slowdown_start = float(
            self.get_parameter('corner_slowdown_start').value)
        self.corner_full_slowdown_angular = float(
            self.get_parameter('corner_full_slowdown_angular').value)
        self.corner_max_forward_velocity = float(
            self.get_parameter('corner_max_forward_velocity').value)
        self.turning_in_place = False

        if self.angular_exit_threshold >= self.angular_enter_threshold:
            raise ValueError(
                'angular_exit_threshold must be smaller than '
                'angular_enter_threshold')
        if self.corner_full_slowdown_angular <= self.corner_slowdown_start:
            raise ValueError(
                'corner_full_slowdown_angular must be greater than '
                'corner_slowdown_start')

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.sub = self.create_subscription(Twist, input_topic, self.cmd_cb, 10)
        self.get_logger().info(
            f'Turn-in-place filter: {input_topic} -> {output_topic}, '
            f'angular_enter={self.angular_enter_threshold:.3f}, '
            f'angular_exit={self.angular_exit_threshold:.3f}, '
            f'hard_stop_angular={self.hard_stop_angular_threshold:.3f}, '
            f'min_forward_keep={self.min_forward_to_keep_turning:.3f}, '
            f'allow_reverse={self.allow_reverse}, '
            f'reverse_max_abs={self.reverse_max_abs:.3f}, '
            f'reverse_block_angular_z={self.reverse_block_angular_z:.3f}, '
            f'max_forward={self.max_forward_velocity:.3f}, '
            f'max_angular={self.max_angular_velocity:.3f}, '
            f'corner_forward={self.corner_max_forward_velocity:.3f}'
        )

    def cmd_cb(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = max(
            -self.max_angular_velocity,
            min(msg.angular.z, self.max_angular_velocity),
        )

        angular_speed = math.fabs(out.angular.z)
        if out.linear.x > 0.0:
            out.linear.x = min(out.linear.x, self.max_forward_velocity)
            if angular_speed > self.corner_slowdown_start:
                turn_ratio = min(
                    1.0,
                    (angular_speed - self.corner_slowdown_start) /
                    (
                        self.corner_full_slowdown_angular -
                        self.corner_slowdown_start
                    ),
                )
                corner_limit = (
                    self.max_forward_velocity -
                    turn_ratio * (
                        self.max_forward_velocity -
                        self.corner_max_forward_velocity
                    )
                )
                out.linear.x = min(out.linear.x, corner_limit)

        if out.linear.x < 0.0:
            if (
                not self.allow_reverse or
                angular_speed >= self.reverse_block_angular_z
            ):
                out.linear.x = 0.0
            else:
                out.linear.x = max(out.linear.x, -abs(self.reverse_max_abs))

        if self.turning_in_place:
            if angular_speed <= self.angular_exit_threshold:
                self.turning_in_place = False
        elif angular_speed >= self.angular_enter_threshold:
            self.turning_in_place = True

        should_lock_translation = (
            self.turning_in_place and
            angular_speed >= self.hard_stop_angular_threshold and
            out.linear.x <= self.min_forward_to_keep_turning
        )

        if should_lock_translation:
            out.linear.x = 0.0
            if self.zero_y_when_turning:
                out.linear.y = 0.0

        self.pub.publish(out)


def main():
    rclpy.init()
    node = TurnInPlaceCmdFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
