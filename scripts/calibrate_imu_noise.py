#!/usr/bin/env python3
import argparse
import json
import math
import os
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def mean(values):
    return statistics.fmean(values) if values else 0.0


def stdev(values):
    return statistics.stdev(values) if len(values) > 1 else 0.0


class ImuNoiseCollector(Node):
    def __init__(self, topic, duration_sec):
        super().__init__("calibrate_imu_noise")
        self.topic = topic
        self.duration_sec = duration_sec
        self.samples = []
        self.t_start_wall = time.monotonic()
        self.create_subscription(Imu, topic, self.imu_cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"Collecting stationary IMU samples from {topic} for {duration_sec:.1f} s")

    def imu_cb(self, msg):
        self.samples.append({
            "t": stamp_to_sec(msg.header.stamp),
            "gx": msg.angular_velocity.x,
            "gy": msg.angular_velocity.y,
            "gz": msg.angular_velocity.z,
            "ax": msg.linear_acceleration.x,
            "ay": msg.linear_acceleration.y,
            "az": msg.linear_acceleration.z,
        })

    def done(self):
        return (time.monotonic() - self.t_start_wall) >= self.duration_sec


def summarize(samples):
    if len(samples) < 3:
      raise RuntimeError("Not enough IMU samples")

    times = [s["t"] for s in samples]
    dts = [b - a for a, b in zip(times[:-1], times[1:]) if b > a]
    dt_avg = mean(dts)
    hz = 1.0 / dt_avg if dt_avg > 0.0 else 0.0

    fields = ["gx", "gy", "gz", "ax", "ay", "az"]
    stats = {}
    for field in fields:
        values = [s[field] for s in samples]
        stats[field] = {
            "mean": mean(values),
            "std_per_sample": stdev(values),
        }

    gyro_std = mean([stats[k]["std_per_sample"] for k in ["gx", "gy", "gz"]])
    acc_std = mean([stats[k]["std_per_sample"] for k in ["ax", "ay", "az"]])

    # ORB-SLAM3 expects continuous-time white noise density. For stationary,
    # evenly sampled white noise, density ~= per-sample std * sqrt(dt).
    noise_gyro = gyro_std * math.sqrt(dt_avg) if dt_avg > 0.0 else 0.0
    noise_acc = acc_std * math.sqrt(dt_avg) if dt_avg > 0.0 else 0.0

    accel_norm = [
        math.sqrt(s["ax"] * s["ax"] + s["ay"] * s["ay"] + s["az"] * s["az"])
        for s in samples
    ]

    return {
        "sample_count": len(samples),
        "duration_msg_sec": max(times) - min(times),
        "hz": hz,
        "dt_avg_sec": dt_avg,
        "dt_min_sec": min(dts) if dts else 0.0,
        "dt_max_sec": max(dts) if dts else 0.0,
        "axis_stats": stats,
        "accel_norm_mean": mean(accel_norm),
        "accel_norm_std": stdev(accel_norm),
        "orb_slam3_preliminary": {
            "IMU.NoiseGyro": noise_gyro,
            "IMU.NoiseAcc": noise_acc,
            "IMU.GyroWalk": None,
            "IMU.AccWalk": None,
            "note": "NoiseGyro/NoiseAcc are short stationary estimates. GyroWalk/AccWalk require Allan variance from a long static bag.",
        },
    }


def write_outputs(output_dir, metrics):
    os.makedirs(output_dir, exist_ok=True)
    json_path = os.path.join(output_dir, "imu_noise_metrics.json")
    yaml_path = os.path.join(output_dir, "orb_imu_noise_preliminary.yaml")

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)

    prelim = metrics["orb_slam3_preliminary"]
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write("# Preliminary stationary IMU noise estimate.\n")
        f.write("# Do not treat GyroWalk/AccWalk as calibrated without Allan variance.\n")
        f.write(f"IMU.NoiseGyro: {prelim['IMU.NoiseGyro']:.9g}\n")
        f.write(f"IMU.NoiseAcc: {prelim['IMU.NoiseAcc']:.9g}\n")
        f.write("# IMU.GyroWalk: TODO_Allan_variance\n")
        f.write("# IMU.AccWalk: TODO_Allan_variance\n")

    return json_path, yaml_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/imu/data_raw")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--output-dir",
                        default=os.path.expanduser("~/agv_mpc/calib/imu_noise_latest"))
    args = parser.parse_args()

    rclpy.init()
    node = ImuNoiseCollector(args.topic, args.duration)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        metrics = summarize(node.samples)
        json_path, yaml_path = write_outputs(args.output_dir, metrics)
        node.get_logger().info(f"Wrote {json_path}")
        node.get_logger().info(f"Wrote {yaml_path}")
        print(json.dumps(metrics, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
