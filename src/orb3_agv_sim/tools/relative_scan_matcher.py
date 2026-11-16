#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def pose_from_odom(msg: Odometry) -> Tuple[float, float, float]:
    p = msg.pose.pose.position
    return (float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation))


def pose_between(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    ax, ay, ath = a
    bx, by, bth = b
    dx_w = bx - ax
    dy_w = by - ay
    c = math.cos(ath)
    s = math.sin(ath)
    return (
        c * dx_w + s * dy_w,
        -s * dx_w + c * dy_w,
        wrap_angle(bth - ath),
    )


def pose_compose(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    ax, ay, ath = a
    bx, by, bth = b
    c = math.cos(ath)
    s = math.sin(ath)
    return (
        ax + c * bx - s * by,
        ay + s * bx + c * by,
        wrap_angle(ath + bth),
    )


def pose_inverse(a: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, th = a
    c = math.cos(th)
    s = math.sin(th)
    return (-c * x - s * y, s * x - c * y, -th)


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def transform_points(points: np.ndarray, pose: Tuple[float, float, float]) -> np.ndarray:
    x, y, th = pose
    c = math.cos(th)
    s = math.sin(th)
    out = np.empty_like(points)
    out[:, 0] = c * points[:, 0] - s * points[:, 1] + x
    out[:, 1] = s * points[:, 0] + c * points[:, 1] + y
    return out


def rigid_transform_2d(src: np.ndarray, dst: np.ndarray) -> Optional[Tuple[float, float, float]]:
    if src.shape[0] < 3:
        return None
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean
    h = src_c.T @ dst_c
    try:
        u, _, vt = np.linalg.svd(h)
    except np.linalg.LinAlgError:
        return None
    r = vt.T @ u.T
    if np.linalg.det(r) < 0.0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    t = dst_mean - r @ src_mean
    yaw = math.atan2(r[1, 0], r[0, 0])
    return (float(t[0]), float(t[1]), wrap_angle(float(yaw)))


class RelativeScanMatcher(Node):
    def __init__(self):
        super().__init__("relative_scan_matcher")

        self.scan_topic = self.declare_parameter("scan_topic", "/scan_filtered").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odometry/filtered").value
        self.output_topic = self.declare_parameter("output_topic", "/scan_match/odom").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "Base_footprint").value
        self.use_odom_initial_guess = as_bool(
            self.declare_parameter("use_odom_initial_guess", True).value)
        self.publish_on_icp_failure = as_bool(
            self.declare_parameter("publish_on_icp_failure", False).value)

        self.min_range = float(self.declare_parameter("min_range", 0.08).value)
        self.max_range = float(self.declare_parameter("max_range", 8.0).value)
        self.angle_stride = int(self.declare_parameter("angle_stride", 2).value)
        self.max_points = int(self.declare_parameter("max_points", 360).value)
        self.min_points = int(self.declare_parameter("min_points", 80).value)
        self.min_correspondences = int(self.declare_parameter("min_correspondences", 45).value)
        self.max_correspondence_dist = float(self.declare_parameter("max_correspondence_dist", 0.35).value)
        self.icp_iterations = int(self.declare_parameter("icp_iterations", 12).value)
        self.min_scan_dt = float(self.declare_parameter("min_scan_dt", 0.08).value)
        self.max_scan_dt = float(self.declare_parameter("max_scan_dt", 1.0).value)
        self.max_delta_xy = float(self.declare_parameter("max_delta_xy", 1.0).value)
        self.max_delta_yaw = float(self.declare_parameter("max_delta_yaw", 0.8).value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_odom: Optional[Tuple[float, float, float]] = None
        self.latest_odom_stamp = -1.0
        self.prev_odom_at_scan: Optional[Tuple[float, float, float]] = None
        self.prev_points: Optional[np.ndarray] = None
        self.prev_stamp = -1.0
        self.last_low_points_warn = -1.0
        self.last_icp_warn = -1.0
        self.scan_odom_pose = (0.0, 0.0, 0.0)
        self.base_to_scan: Optional[Tuple[float, float, float]] = None
        self.warned_no_tf = False

        self.pub = self.create_publisher(Odometry, self.output_topic, 20)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)

        self.get_logger().info(
            f"Relative scan matcher: scan={self.scan_topic}, odom_guess={self.odom_topic}, output={self.output_topic}"
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = pose_from_odom(msg)
        self.latest_odom_stamp = self.stamp_to_sec(msg.header.stamp)

    @staticmethod
    def stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def scan_to_points(self, msg: LaserScan) -> Optional[np.ndarray]:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return None
        idx = np.arange(0, ranges.size, max(1, self.angle_stride), dtype=np.int32)
        rr = ranges[idx]
        angles = float(msg.angle_min) + idx.astype(np.float32) * float(msg.angle_increment)
        valid = np.isfinite(rr) & (rr >= self.min_range) & (rr <= min(self.max_range, float(msg.range_max)))
        rr = rr[valid]
        angles = angles[valid]
        if rr.size < self.min_points:
            return None
        if rr.size > self.max_points:
            sel = np.linspace(0, rr.size - 1, self.max_points).astype(np.int32)
            rr = rr[sel]
            angles = angles[sel]
        return np.column_stack((rr * np.cos(angles), rr * np.sin(angles))).astype(np.float32)

    def lookup_base_to_scan(self, scan_frame: str) -> Tuple[float, float, float]:
        if self.base_to_scan is not None:
            return self.base_to_scan
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame, scan_frame, Time(), timeout=Duration(seconds=0.05)
            )
            tr = tf.transform.translation
            self.base_to_scan = (float(tr.x), float(tr.y), yaw_from_quat(tf.transform.rotation))
            self.get_logger().info(
                f"Using TF {self.base_frame}->{scan_frame}: x={self.base_to_scan[0]:.3f} "
                f"y={self.base_to_scan[1]:.3f} yaw={self.base_to_scan[2]:.3f}"
            )
        except TransformException as ex:
            self.base_to_scan = (0.0, 0.0, 0.0)
            if not self.warned_no_tf:
                self.warned_no_tf = True
                self.get_logger().warn(
                    f"Cannot lookup {self.base_frame}->{scan_frame}: {ex}. "
                    "Using zero laser offset for relative scan odom."
                )
        return self.base_to_scan

    def refine_delta_icp(
        self,
        current_points: np.ndarray,
        previous_points: np.ndarray,
        initial_delta: Tuple[float, float, float],
    ) -> Optional[Tuple[float, float, float, int, float]]:
        delta = initial_delta
        mean_err = float("inf")
        corr_count = 0
        prev = previous_points.astype(np.float32)
        curr = current_points.astype(np.float32)

        for _ in range(max(1, self.icp_iterations)):
            curr_tf = transform_points(curr, delta)
            diff = curr_tf[:, None, :] - prev[None, :, :]
            dist2 = np.einsum("ijk,ijk->ij", diff, diff)
            nearest = np.argmin(dist2, axis=1)
            best_dist = np.sqrt(dist2[np.arange(dist2.shape[0]), nearest])
            mask = best_dist < self.max_correspondence_dist
            corr_count = int(np.count_nonzero(mask))
            if corr_count < self.min_correspondences:
                return None
            matched_src = curr[mask]
            matched_dst = prev[nearest[mask]]
            solved = rigid_transform_2d(matched_src, matched_dst)
            if solved is None:
                return None
            step = pose_between(delta, solved)
            delta = solved
            mean_err = float(np.mean(best_dist[mask]))
            if abs(step[0]) < 0.001 and abs(step[1]) < 0.001 and abs(step[2]) < 0.001:
                break

        return (delta[0], delta[1], delta[2], corr_count, mean_err)

    def publish_odom(self, stamp, delta_base: Tuple[float, float, float], covariance_scale: float):
        self.scan_odom_pose = pose_compose(self.scan_odom_pose, delta_base)
        x, y, yaw = self.scan_odom_pose
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        sx = max(0.02, 0.05 * covariance_scale)
        sy = max(0.02, 0.05 * covariance_scale)
        syaw = max(0.02, 0.04 * covariance_scale)
        msg.pose.covariance[0] = sx * sx
        msg.pose.covariance[7] = sy * sy
        msg.pose.covariance[35] = syaw * syaw
        self.pub.publish(msg)

    def scan_callback(self, msg: LaserScan):
        stamp = self.stamp_to_sec(msg.header.stamp)
        points = self.scan_to_points(msg)
        if points is None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_low_points_warn > 2.0:
                self.last_low_points_warn = now
                self.get_logger().warn("Reject scan matching: too few valid scan points")
            return

        if self.prev_points is None:
            self.prev_points = points
            self.prev_stamp = stamp
            self.prev_odom_at_scan = self.latest_odom
            self.publish_odom(msg.header.stamp, (0.0, 0.0, 0.0), 1.0)
            return

        dt = stamp - self.prev_stamp
        if dt < self.min_scan_dt:
            return
        if dt > self.max_scan_dt:
            self.get_logger().warn(f"Reset scan matching chain: scan gap {dt:.3f}s > {self.max_scan_dt:.3f}s")
            self.prev_points = points
            self.prev_stamp = stamp
            self.prev_odom_at_scan = self.latest_odom
            return

        initial_delta = (0.0, 0.0, 0.0)
        if self.use_odom_initial_guess and self.prev_odom_at_scan is not None and self.latest_odom is not None:
            initial_delta = pose_between(self.prev_odom_at_scan, self.latest_odom)

        result = self.refine_delta_icp(points, self.prev_points, initial_delta)
        if result is None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_icp_warn > 2.0:
                self.last_icp_warn = now
                self.get_logger().warn("Reject scan matching: ICP did not converge")
            if not self.publish_on_icp_failure:
                return
            delta_laser = initial_delta
            corr_count = 0
            mean_err = 1.0
        else:
            delta_laser = (result[0], result[1], result[2])
            corr_count = result[3]
            mean_err = result[4]

        dxy = math.hypot(delta_laser[0], delta_laser[1])
        dyaw = abs(wrap_angle(delta_laser[2]))
        if dxy > self.max_delta_xy or dyaw > self.max_delta_yaw:
            self.get_logger().warn(
                f"Reject scan matching jump: dxy={dxy:.3f}/{self.max_delta_xy:.3f}, "
                f"dyaw={dyaw:.3f}/{self.max_delta_yaw:.3f}"
            )
            return

        base_to_scan = self.lookup_base_to_scan(msg.header.frame_id)
        delta_base = pose_compose(pose_compose(base_to_scan, delta_laser), pose_inverse(base_to_scan))
        covariance_scale = 1.0 + min(5.0, mean_err / max(1e-3, self.max_correspondence_dist))
        self.publish_odom(msg.header.stamp, delta_base, covariance_scale)

        self.prev_points = points
        self.prev_stamp = stamp
        self.prev_odom_at_scan = self.latest_odom

        self.get_logger().info(
            f"scan_match delta base dx={delta_base[0]:.3f} dy={delta_base[1]:.3f} "
            f"dyaw={delta_base[2]:.3f} corr={corr_count} err={mean_err:.3f}"
        )


def main():
    rclpy.init()
    node = RelativeScanMatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
