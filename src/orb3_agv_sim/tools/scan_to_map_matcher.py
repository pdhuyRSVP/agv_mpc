#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def transform_points(points: np.ndarray, pose: Tuple[float, float, float]) -> np.ndarray:
    x, y, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    result = np.empty_like(points)
    result[:, 0] = c * points[:, 0] - s * points[:, 1] + x
    result[:, 1] = s * points[:, 0] + c * points[:, 1] + y
    return result


class ScanToMapMatcher(Node):
    def __init__(self):
        super().__init__("scan_to_map_matcher")
        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.scan_topic = self.declare_parameter("scan_topic", "/scan_filtered").value
        self.initial_pose_topic = self.declare_parameter(
            "initial_pose_topic", "/graph/pose").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/scan_match/map_pose").value
        self.initialpose_topic = self.declare_parameter(
            "initialpose_topic", "/initialpose").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "Base_footprint").value
        initial_x = float(self.declare_parameter("initial_x", 0.0).value)
        initial_y = float(self.declare_parameter("initial_y", 0.0).value)
        initial_yaw = float(self.declare_parameter("initial_yaw", 0.0).value)

        self.min_range = float(self.declare_parameter("min_range", 0.20).value)
        self.max_range = float(self.declare_parameter("max_range", 8.0).value)
        self.angle_stride = int(self.declare_parameter("angle_stride", 3).value)
        self.max_points = int(self.declare_parameter("max_points", 240).value)
        self.min_points = int(self.declare_parameter("min_points", 55).value)
        self.min_update_period = float(
            self.declare_parameter("min_update_period", 0.30).value)

        self.search_xy = float(self.declare_parameter("search_xy", 0.40).value)
        self.search_yaw = float(self.declare_parameter(
            "search_yaw", math.radians(15.0)).value)
        self.coarse_xy_step = float(
            self.declare_parameter("coarse_xy_step", 0.10).value)
        self.coarse_yaw_step = float(self.declare_parameter(
            "coarse_yaw_step", math.radians(3.0)).value)
        self.fine_xy_step = float(self.declare_parameter("fine_xy_step", 0.025).value)
        self.fine_yaw_step = float(self.declare_parameter(
            "fine_yaw_step", math.radians(0.75)).value)

        self.max_mean_distance = float(
            self.declare_parameter("max_mean_distance", 0.35).value)
        self.min_valid_fraction = float(
            self.declare_parameter("min_valid_fraction", 0.60).value)
        self.max_correction_xy = float(
            self.declare_parameter("max_correction_xy", 0.70).value)
        self.max_correction_yaw = float(self.declare_parameter(
            "max_correction_yaw", math.radians(20.0)).value)
        self.translation_prior_weight = float(
            self.declare_parameter("translation_prior_weight", 0.10).value)
        self.yaw_prior_weight = float(
            self.declare_parameter("yaw_prior_weight", 0.05).value)
        self.occupied_threshold = int(
            self.declare_parameter("occupied_threshold", 65).value)

        self.map_distance: Optional[np.ndarray] = None
        self.map_resolution = 0.0
        self.map_origin = (0.0, 0.0, 0.0)
        self.predicted_pose: Optional[Tuple[float, float, float]] = (
            initial_x, initial_y, initial_yaw)
        self.base_to_scan: Optional[Tuple[float, float, float]] = None
        self.last_processed_stamp = -1.0
        self.accepted = 0
        self.rejected = 0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.output_topic, 10)
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(PoseStamped, self.initial_pose_topic, self.pose_callback, 20)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initialpose_callback,
            10,
        )
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info(
            f"Scan-to-map matcher: map={self.map_topic}, scan={self.scan_topic}, "
            f"guess={self.initial_pose_topic}, output={self.output_topic}, "
            f"seed=({initial_x:.3f},{initial_y:.3f},{initial_yaw:.3f})")

    @staticmethod
    def stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def map_callback(self, msg: OccupancyGrid):
        width = int(msg.info.width)
        height = int(msg.info.height)
        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        occupied = data >= self.occupied_threshold
        distance_input = np.where(occupied, 0, 255).astype(np.uint8)
        distance_px = cv2.distanceTransform(
            distance_input, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        self.map_distance = distance_px * float(msg.info.resolution)
        self.map_resolution = float(msg.info.resolution)
        origin = msg.info.origin
        self.map_origin = (
            float(origin.position.x),
            float(origin.position.y),
            yaw_from_quat(origin.orientation),
        )
        self.get_logger().info(
            f"Loaded occupancy distance field {width}x{height}, "
            f"resolution={self.map_resolution:.3f} m")

    def pose_callback(self, msg: PoseStamped):
        p = msg.pose.position
        self.predicted_pose = (
            float(p.x), float(p.y), yaw_from_quat(msg.pose.orientation))

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.predicted_pose = (
            float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation))
        self.get_logger().info(
            "Updated scan-to-map seed from /initialpose: "
            f"({self.predicted_pose[0]:.3f},{self.predicted_pose[1]:.3f},"
            f"{self.predicted_pose[2]:.3f})")

    def scan_points_in_base(self, msg: LaserScan) -> Optional[np.ndarray]:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        indices = np.arange(
            0, ranges.size, max(1, self.angle_stride), dtype=np.int32)
        values = ranges[indices]
        angles = float(msg.angle_min) + indices * float(msg.angle_increment)
        valid = (
            np.isfinite(values) &
            (values >= max(self.min_range, float(msg.range_min))) &
            (values <= min(self.max_range, float(msg.range_max)))
        )
        values = values[valid]
        angles = angles[valid]
        if values.size < self.min_points:
            return None
        if values.size > self.max_points:
            selected = np.linspace(
                0, values.size - 1, self.max_points).astype(np.int32)
            values = values[selected]
            angles = angles[selected]
        scan_points = np.column_stack((
            values * np.cos(angles),
            values * np.sin(angles),
        )).astype(np.float32)
        return transform_points(scan_points, self.lookup_base_to_scan(msg.header.frame_id))

    def lookup_base_to_scan(self, scan_frame: str) -> Tuple[float, float, float]:
        if self.base_to_scan is not None:
            return self.base_to_scan
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame, scan_frame, Time(),
                timeout=Duration(seconds=0.10))
            tr = tf.transform.translation
            self.base_to_scan = (
                float(tr.x), float(tr.y), yaw_from_quat(tf.transform.rotation))
            self.get_logger().info(
                f"Using TF {self.base_frame}->{scan_frame}: "
                f"x={self.base_to_scan[0]:.3f}, y={self.base_to_scan[1]:.3f}, "
                f"yaw={self.base_to_scan[2]:.3f}")
        except TransformException as ex:
            self.get_logger().warn(
                f"Cannot match scan without {self.base_frame}->{scan_frame}: {ex}")
            raise
        return self.base_to_scan

    def score_pose(
        self, points_base: np.ndarray, pose: Tuple[float, float, float]
    ) -> Tuple[float, float]:
        points_map = transform_points(points_base, pose)
        ox, oy, oyaw = self.map_origin
        c = math.cos(oyaw)
        s = math.sin(oyaw)
        dx = points_map[:, 0] - ox
        dy = points_map[:, 1] - oy
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        cols = np.floor(local_x / self.map_resolution).astype(np.int32)
        rows = np.floor(local_y / self.map_resolution).astype(np.int32)
        height, width = self.map_distance.shape
        valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        valid_fraction = float(np.mean(valid))
        if not np.any(valid):
            return float("inf"), 0.0
        distances = self.map_distance[rows[valid], cols[valid]]
        capped = np.minimum(distances, 1.0)
        score = float(np.mean(capped) + (1.0 - valid_fraction) * 0.5)
        return score, valid_fraction

    @staticmethod
    def offsets(radius: float, step: float) -> np.ndarray:
        count = max(1, int(math.ceil(radius / step)))
        return np.arange(-count, count + 1, dtype=np.float64) * step

    def search(
        self, points: np.ndarray, center: Tuple[float, float, float],
        xy_radius: float, yaw_radius: float, xy_step: float, yaw_step: float
    ) -> Tuple[Tuple[float, float, float], float, float]:
        best_pose = center
        best_score, best_fraction = self.score_pose(points, center)
        best_objective = best_score
        for dyaw in self.offsets(yaw_radius, yaw_step):
            yaw = wrap_angle(center[2] + float(dyaw))
            for dx in self.offsets(xy_radius, xy_step):
                for dy in self.offsets(xy_radius, xy_step):
                    candidate = (center[0] + float(dx), center[1] + float(dy), yaw)
                    score, fraction = self.score_pose(points, candidate)
                    objective = (
                        score +
                        self.translation_prior_weight * math.hypot(
                            candidate[0] - center[0],
                            candidate[1] - center[1],
                        ) +
                        self.yaw_prior_weight * abs(
                            wrap_angle(candidate[2] - center[2])
                        )
                    )
                    if objective < best_objective:
                        best_pose = candidate
                        best_score = score
                        best_fraction = fraction
                        best_objective = objective
        return best_pose, best_score, best_fraction

    def scan_callback(self, msg: LaserScan):
        if self.map_distance is None or self.predicted_pose is None:
            return
        stamp = self.stamp_to_sec(msg.header.stamp)
        if stamp - self.last_processed_stamp < self.min_update_period:
            return
        self.last_processed_stamp = stamp

        try:
            points = self.scan_points_in_base(msg)
        except TransformException:
            self.rejected += 1
            return
        if points is None:
            self.rejected += 1
            return

        coarse_pose, _, _ = self.search(
            points, self.predicted_pose,
            self.search_xy, self.search_yaw,
            self.coarse_xy_step, self.coarse_yaw_step)
        best_pose, score, valid_fraction = self.search(
            points, coarse_pose,
            self.coarse_xy_step, self.coarse_yaw_step,
            self.fine_xy_step, self.fine_yaw_step)

        correction_xy = math.hypot(
            best_pose[0] - self.predicted_pose[0],
            best_pose[1] - self.predicted_pose[1])
        correction_yaw = abs(wrap_angle(best_pose[2] - self.predicted_pose[2]))
        if (
            score > self.max_mean_distance or
            valid_fraction < self.min_valid_fraction or
            correction_xy > self.max_correction_xy or
            correction_yaw > self.max_correction_yaw
        ):
            self.rejected += 1
            self.get_logger().warn(
                f"Reject scan-map: score={score:.3f}/{self.max_mean_distance:.3f}, "
                f"valid={valid_fraction:.2f}/{self.min_valid_fraction:.2f}, "
                f"correction={correction_xy:.3f}m,{math.degrees(correction_yaw):.1f}deg")
            return

        sigma_xy = max(0.04, min(0.30, score * 0.8))
        sigma_yaw = max(
            math.radians(1.5),
            min(math.radians(12.0), score * 0.5 + correction_yaw * 0.25))
        output = PoseWithCovarianceStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.map_frame
        output.pose.pose.position.x = best_pose[0]
        output.pose.pose.position.y = best_pose[1]
        output.pose.pose.orientation.z = math.sin(best_pose[2] * 0.5)
        output.pose.pose.orientation.w = math.cos(best_pose[2] * 0.5)
        output.pose.covariance[0] = sigma_xy * sigma_xy
        output.pose.covariance[7] = sigma_xy * sigma_xy
        output.pose.covariance[35] = sigma_yaw * sigma_yaw
        self.predicted_pose = best_pose
        self.publisher.publish(output)
        self.accepted += 1
        self.get_logger().info(
            f"scan-map accepted pose=({best_pose[0]:.3f},{best_pose[1]:.3f},"
            f"{best_pose[2]:.3f}) score={score:.3f} valid={valid_fraction:.2f} "
            f"accepted/rejected={self.accepted}/{self.rejected}")


def main():
    rclpy.init()
    node = ScanToMapMatcher()
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
