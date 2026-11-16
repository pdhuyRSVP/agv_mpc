#!/usr/bin/env python3
import csv
import math
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class CsvSink:
    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.file = open(path, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "wall_time_sec",
            "stamp_sec",
            "source",
            "frame_id",
            "x",
            "y",
            "z",
            "qx",
            "qy",
            "qz",
            "qw",
            "yaw_rad",
        ])
        self.file.flush()

    def write_pose(self, wall_time_sec, stamp_sec, source, frame_id, pose):
        q = pose.orientation
        p = pose.position
        self.writer.writerow([
            f"{wall_time_sec:.9f}",
            f"{stamp_sec:.9f}",
            source,
            frame_id,
            f"{p.x:.9f}",
            f"{p.y:.9f}",
            f"{p.z:.9f}",
            f"{q.x:.9f}",
            f"{q.y:.9f}",
            f"{q.z:.9f}",
            f"{q.w:.9f}",
            f"{yaw_from_quaternion(q):.9f}",
        ])
        self.file.flush()

    def close(self):
        self.file.close()


class TrajectoryCompareLogger(Node):
    def __init__(self):
        super().__init__("trajectory_compare_logger")

        default_dir = os.path.join(
            os.path.expanduser("~"),
            "agv_mpc",
            "trajectory_logs",
            datetime.now().strftime("%Y%m%d_%H%M%S"),
        )
        self.output_dir = os.path.expanduser(
            self.declare_parameter("output_dir", default_dir).value or default_dir)
        self.orb_rgbd_pose_topic = self.declare_parameter(
            "orb_rgbd_pose_topic", "/orb_rgbd/pose").value
        self.orb_rgbd_imu_pose_topic = self.declare_parameter(
            "orb_rgbd_imu_pose_topic", "/orb_rgbd_imu/pose").value
        self.amcl_pose_topic = self.declare_parameter("amcl_pose_topic", "/amcl_pose").value
        self.slam_toolbox_pose_topic = self.declare_parameter(
            "slam_toolbox_pose_topic", "/slam_toolbox/pose").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.fused_pose_topic = self.declare_parameter(
            "fused_pose_topic", "/graph/pose").value
        self.enable_amcl_tf = self.declare_parameter("enable_amcl_tf", True).value
        self.amcl_map_frame = self.declare_parameter("amcl_map_frame", "map").value
        self.amcl_base_frame = self.declare_parameter("amcl_base_frame", "Base_footprint").value
        self.amcl_tf_hz = float(self.declare_parameter("amcl_tf_hz", 5.0).value)

        self.sinks = {
            "orb_rgbd": CsvSink(os.path.join(self.output_dir, "orb_rgbd.csv")),
            "orb_rgbd_imu": CsvSink(os.path.join(self.output_dir, "orb_rgbd_imu.csv")),
            "amcl": CsvSink(os.path.join(self.output_dir, "amcl.csv")),
            "amcl_tf": CsvSink(os.path.join(self.output_dir, "amcl_tf.csv")),
            "slam_toolbox": CsvSink(os.path.join(self.output_dir, "slam_toolbox.csv")),
            "odom": CsvSink(os.path.join(self.output_dir, "odom.csv")),
            "graph_fusion": CsvSink(os.path.join(self.output_dir, "graph_fusion.csv")),
        }

        self.create_subscription(
            PoseStamped,
            self.orb_rgbd_pose_topic,
            lambda msg: self.pose_stamped_cb("orb_rgbd", msg),
            50,
        )
        self.create_subscription(
            PoseStamped,
            self.orb_rgbd_imu_pose_topic,
            lambda msg: self.pose_stamped_cb("orb_rgbd_imu", msg),
            50,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.amcl_cb,
            50,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.slam_toolbox_pose_topic,
            self.slam_toolbox_cb,
            50,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            50,
        )
        self.create_subscription(
            PoseStamped,
            self.fused_pose_topic,
            lambda msg: self.pose_stamped_cb("graph_fusion", msg),
            50,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.enable_amcl_tf and self.amcl_tf_hz > 0.0:
            self.create_timer(1.0 / self.amcl_tf_hz, self.amcl_tf_cb)

        self.get_logger().info(f"Trajectory CSV output: {self.output_dir}")
        self.get_logger().info(f"ORB RGB-D pose topic: {self.orb_rgbd_pose_topic}")
        self.get_logger().info(f"ORB RGB-D+IMU pose topic: {self.orb_rgbd_imu_pose_topic}")
        self.get_logger().info(f"AMCL pose topic: {self.amcl_pose_topic}")
        self.get_logger().info(f"slam_toolbox pose topic: {self.slam_toolbox_pose_topic}")
        self.get_logger().info(f"Odometry topic: {self.odom_topic}")
        self.get_logger().info(f"GTSAM fused pose topic: {self.fused_pose_topic}")

    def pose_stamped_cb(self, source, msg):
        self.sinks[source].write_pose(
            self.get_clock().now().nanoseconds * 1e-9,
            stamp_to_sec(msg.header.stamp),
            source,
            msg.header.frame_id,
            msg.pose,
        )

    def amcl_cb(self, msg):
        self.sinks["amcl"].write_pose(
            self.get_clock().now().nanoseconds * 1e-9,
            stamp_to_sec(msg.header.stamp),
            "amcl",
            msg.header.frame_id,
            msg.pose.pose,
        )

    def slam_toolbox_cb(self, msg):
        self.sinks["slam_toolbox"].write_pose(
            self.get_clock().now().nanoseconds * 1e-9,
            stamp_to_sec(msg.header.stamp),
            "slam_toolbox",
            msg.header.frame_id,
            msg.pose.pose,
        )

    def odom_cb(self, msg):
        self.sinks["odom"].write_pose(
            self.get_clock().now().nanoseconds * 1e-9,
            stamp_to_sec(msg.header.stamp),
            "odom",
            msg.header.frame_id,
            msg.pose.pose,
        )

    def amcl_tf_cb(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.amcl_map_frame,
                self.amcl_base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().debug(f"Waiting AMCL TF {self.amcl_map_frame}->{self.amcl_base_frame}: {exc}")
            return

        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        self.pose_stamped_cb("amcl_tf", pose)

    def destroy_node(self):
        for sink in self.sinks.values():
            sink.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TrajectoryCompareLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
