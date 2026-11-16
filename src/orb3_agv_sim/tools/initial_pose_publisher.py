#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_publisher")
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.x = float(self.declare_parameter("x", 0.0).value)
        self.y = float(self.declare_parameter("y", 0.0).value)
        self.yaw = float(self.declare_parameter("yaw", 0.0).value)
        self.cov_xy = float(self.declare_parameter("cov_xy", 0.25).value)
        self.cov_yaw = float(self.declare_parameter("cov_yaw", 0.0685).value)
        self.publish_count = int(self.declare_parameter("publish_count", 5).value)
        self.period_sec = float(self.declare_parameter("period_sec", 0.5).value)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.sent = 0
        self.timer = self.create_timer(self.period_sec, self.publish_pose)
        self.get_logger().info(
            f"Initial pose publisher: frame={self.frame_id}, x={self.x:.3f}, "
            f"y={self.y:.3f}, yaw={self.yaw:.3f}, count={self.publish_count}")

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        msg.pose.covariance[0] = self.cov_xy
        msg.pose.covariance[7] = self.cov_xy
        msg.pose.covariance[35] = self.cov_yaw
        self.pub.publish(msg)

        self.sent += 1
        if self.sent >= self.publish_count:
            self.get_logger().info("Initial pose published")
            self.timer.cancel()


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
