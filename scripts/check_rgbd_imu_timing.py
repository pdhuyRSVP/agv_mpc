#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def hz_from_times(times):
    if len(times) < 2:
        return 0.0
    dt = times[-1] - times[0]
    if dt <= 0.0:
        return 0.0
    return float(len(times) - 1) / dt


class RgbdImuTimingCheck(Node):
    def __init__(self):
        super().__init__("check_rgbd_imu_timing")
        self.color_topic = self.declare_parameter(
            "color_topic", "/camera/color/image_raw").value
        self.depth_topic = self.declare_parameter(
            "depth_topic", "/camera/depth/image_raw").value
        self.imu_topic = self.declare_parameter(
            "imu_topic", "/imu/data_raw").value

        self.color_times = deque(maxlen=200)
        self.depth_times = deque(maxlen=200)
        self.imu_times = deque(maxlen=2000)
        self.imu_per_color = deque(maxlen=100)
        self.rgb_depth_dt_ms = deque(maxlen=100)
        self.last_depth_stamp = None
        self.last_color_stamp = None

        self.create_subscription(
            Image, self.color_topic, self.color_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.depth_topic, self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.imu_topic, self.imu_cb, qos_profile_sensor_data)
        self.create_timer(2.0, self.report)

        self.get_logger().info(f"color_topic: {self.color_topic}")
        self.get_logger().info(f"depth_topic: {self.depth_topic}")
        self.get_logger().info(f"imu_topic:   {self.imu_topic}")

    def color_cb(self, msg):
        t = stamp_to_sec(msg.header.stamp)
        self.color_times.append(t)

        if self.last_color_stamp is not None:
            count = sum(1 for ti in self.imu_times if self.last_color_stamp < ti <= t)
            self.imu_per_color.append(count)
        self.last_color_stamp = t

        if self.last_depth_stamp is not None:
            self.rgb_depth_dt_ms.append(abs(t - self.last_depth_stamp) * 1000.0)

    def depth_cb(self, msg):
        t = stamp_to_sec(msg.header.stamp)
        self.depth_times.append(t)
        self.last_depth_stamp = t

    def imu_cb(self, msg):
        self.imu_times.append(stamp_to_sec(msg.header.stamp))

    def report(self):
        color_hz = hz_from_times(self.color_times)
        depth_hz = hz_from_times(self.depth_times)
        imu_hz = hz_from_times(self.imu_times)

        if self.rgb_depth_dt_ms:
            dt_avg = sum(self.rgb_depth_dt_ms) / len(self.rgb_depth_dt_ms)
            dt_min = min(self.rgb_depth_dt_ms)
            dt_max = max(self.rgb_depth_dt_ms)
        else:
            dt_min = dt_avg = dt_max = math.nan

        if self.imu_per_color:
            imu_min = min(self.imu_per_color)
            imu_avg = sum(self.imu_per_color) / len(self.imu_per_color)
            imu_max = max(self.imu_per_color)
        else:
            imu_min = imu_avg = imu_max = math.nan

        self.get_logger().info(
            "Hz color=%.2f depth=%.2f imu=%.2f | rgb-depth dt ms min/avg/max=%.1f/%.1f/%.1f | IMU/frame min/avg/max=%.1f/%.1f/%.1f",
            color_hz, depth_hz, imu_hz,
            dt_min, dt_avg, dt_max,
            imu_min, imu_avg, imu_max)


def main():
    rclpy.init()
    node = RgbdImuTimingCheck()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
