#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw: float) -> Tuple[float, float]:
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


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


class GlobalScanInitializer(Node):
    """One-shot global LiDAR initializer.

    This node does not publish TF. It searches the 2D occupancy map using the
    current LaserScan and publishes /initialpose for slam_toolbox before the
    graph creates map->odom.
    """

    def __init__(self):
        super().__init__("global_scan_initializer")
        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.scan_topic = self.declare_parameter("scan_topic", "/scan_filtered").value
        self.initialpose_topic = self.declare_parameter("initialpose_topic", "/initialpose").value
        self.output_pose_topic = self.declare_parameter(
            "output_pose_topic", "/global_scan_initializer/pose").value
        self.status_topic = self.declare_parameter(
            "status_topic", "/global_scan_initializer/status").value
        self.trigger_topic = self.declare_parameter(
            "trigger_topic", "/global_scan_initializer/trigger").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "Base_footprint").value

        self.min_range = float(self.declare_parameter("min_range", 0.20).value)
        self.max_range = float(self.declare_parameter("max_range", 8.0).value)
        self.angle_stride = int(self.declare_parameter("angle_stride", 4).value)
        self.max_points = int(self.declare_parameter("max_points", 160).value)
        self.min_points = int(self.declare_parameter("min_points", 45).value)

        self.global_xy_step = float(self.declare_parameter("global_xy_step", 0.25).value)
        self.global_yaw_step = float(self.declare_parameter(
            "global_yaw_step", math.radians(15.0)).value)
        self.fine_xy_radius = float(self.declare_parameter("fine_xy_radius", 0.30).value)
        self.fine_xy_step = float(self.declare_parameter("fine_xy_step", 0.05).value)
        self.fine_yaw_radius = float(self.declare_parameter(
            "fine_yaw_radius", math.radians(15.0)).value)
        self.fine_yaw_step = float(self.declare_parameter(
            "fine_yaw_step", math.radians(3.0)).value)
        self.max_global_candidates = int(
            self.declare_parameter("max_global_candidates", 12000).value)
        self.score_chunk_size = int(self.declare_parameter("score_chunk_size", 256).value)

        self.occupied_threshold = int(
            self.declare_parameter("occupied_threshold", 65).value)
        self.free_threshold = int(self.declare_parameter("free_threshold", 25).value)
        self.obstacle_clearance = float(
            self.declare_parameter("obstacle_clearance", 0.18).value)
        self.max_best_score = float(self.declare_parameter("max_best_score", 0.22).value)
        self.min_valid_fraction = float(
            self.declare_parameter("min_valid_fraction", 0.70).value)
        self.min_score_margin = float(
            self.declare_parameter("min_score_margin", 0.020).value)
        self.required_successes = int(
            self.declare_parameter("required_successes", 2).value)
        self.ambiguous_required_successes = int(
            self.declare_parameter("ambiguous_required_successes", 4).value)
        self.stable_xy_tolerance = float(
            self.declare_parameter("stable_xy_tolerance", 0.35).value)
        self.stable_yaw_tolerance = float(
            self.declare_parameter("stable_yaw_tolerance", math.radians(12.0)).value)
        self.enable_home_seed = bool(
            self.declare_parameter("enable_home_seed", True).value)
        self.home_x = float(self.declare_parameter("home_x", 0.0).value)
        self.home_y = float(self.declare_parameter("home_y", 0.0).value)
        self.home_seed_radius = float(
            self.declare_parameter("home_seed_radius", 0.80).value)
        self.home_required_successes = int(
            self.declare_parameter("home_required_successes", 3).value)
        self.home_yaw_tolerance = float(
            self.declare_parameter("home_yaw_tolerance", math.radians(20.0)).value)
        self.max_scan_attempts = int(self.declare_parameter("max_scan_attempts", 20).value)
        self.republish_count = int(self.declare_parameter("republish_count", 12).value)
        self.republish_period = float(self.declare_parameter("republish_period", 0.25).value)
        self.max_success_history = int(
            self.declare_parameter("max_success_history", 30).value)

        self.map_distance: Optional[np.ndarray] = None
        self.free_cells: Optional[np.ndarray] = None
        self.map_resolution = 0.0
        self.map_origin = (0.0, 0.0, 0.0)
        self.base_to_scan: Optional[Tuple[float, float, float]] = None
        self.initialized = False
        self.scan_attempts = 0
        self.successes = []
        self.home_successes = []
        self.last_pose: Optional[PoseWithCovarianceStamped] = None
        self.remaining_republishes = 0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Bool, self.trigger_topic, self.trigger_callback, 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.initialpose_topic, 10)
        self.output_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.output_pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.republish_timer = self.create_timer(self.republish_period, self.republish)

        self.get_logger().info(
            f"Global scan initializer: map={self.map_topic}, scan={self.scan_topic}, "
            f"initialpose={self.initialpose_topic}, output={self.output_pose_topic}, "
            f"trigger={self.trigger_topic}")

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def trigger_callback(self, msg: Bool):
        if not msg.data:
            return
        self.initialized = False
        self.scan_attempts = 0
        self.successes = []
        self.home_successes = []
        self.last_pose = None
        self.remaining_republishes = 0
        self.publish_status("relocalization_triggered waiting_for_scan")
        self.get_logger().warn(
            "Relocalization trigger received; restarting global search on /scan_filtered")

    def map_callback(self, msg: OccupancyGrid):
        width = int(msg.info.width)
        height = int(msg.info.height)
        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        occupied = data >= self.occupied_threshold
        free = (data >= 0) & (data <= self.free_threshold)
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

        stride = max(1, int(round(self.global_xy_step / self.map_resolution)))
        free &= self.map_distance >= self.obstacle_clearance
        rows, cols = np.nonzero(free[::stride, ::stride])
        rows = rows * stride
        cols = cols * stride
        cells = np.column_stack((rows, cols)).astype(np.int32)
        if cells.shape[0] > self.max_global_candidates:
            rng = np.random.default_rng(17)
            keep = rng.choice(cells.shape[0], self.max_global_candidates, replace=False)
            cells = cells[keep]
        self.free_cells = cells
        self.get_logger().info(
            f"Loaded global init map {width}x{height}, resolution={self.map_resolution:.3f}, "
            f"candidate_xy={cells.shape[0]}")
        self.publish_status(f"map_ready candidates={cells.shape[0]}")

    def lookup_base_to_scan(self, scan_frame: str) -> Tuple[float, float, float]:
        if self.base_to_scan is not None:
            return self.base_to_scan
        tf: TransformStamped = self.tf_buffer.lookup_transform(
            self.base_frame, scan_frame, Time(), timeout=Duration(seconds=0.20))
        tr = tf.transform.translation
        self.base_to_scan = (
            float(tr.x), float(tr.y), yaw_from_quat(tf.transform.rotation))
        self.get_logger().info(
            f"Using TF {self.base_frame}->{scan_frame}: "
            f"x={self.base_to_scan[0]:.3f}, y={self.base_to_scan[1]:.3f}, "
            f"yaw={self.base_to_scan[2]:.3f}")
        return self.base_to_scan

    def scan_points_in_base(self, msg: LaserScan) -> Optional[np.ndarray]:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        indices = np.arange(0, ranges.size, max(1, self.angle_stride), dtype=np.int32)
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
            selected = np.linspace(0, values.size - 1, self.max_points).astype(np.int32)
            values = values[selected]
            angles = angles[selected]
        scan_points = np.column_stack((
            values * np.cos(angles),
            values * np.sin(angles),
        )).astype(np.float32)
        return transform_points(scan_points, self.lookup_base_to_scan(msg.header.frame_id))

    def map_xy_from_cells(self, cells: np.ndarray) -> np.ndarray:
        rows = cells[:, 0].astype(np.float64)
        cols = cells[:, 1].astype(np.float64)
        ox, oy, oyaw = self.map_origin
        local_x = (cols + 0.5) * self.map_resolution
        local_y = (rows + 0.5) * self.map_resolution
        c = math.cos(oyaw)
        s = math.sin(oyaw)
        x = ox + c * local_x - s * local_y
        y = oy + s * local_x + c * local_y
        return np.column_stack((x, y)).astype(np.float32)

    def score_candidates(
        self, points_base: np.ndarray, xy_candidates: np.ndarray, yaw: float
    ) -> Tuple[int, float, float, float]:
        c = math.cos(yaw)
        s = math.sin(yaw)
        offsets = np.empty_like(points_base)
        offsets[:, 0] = c * points_base[:, 0] - s * points_base[:, 1]
        offsets[:, 1] = s * points_base[:, 0] + c * points_base[:, 1]
        best_index = -1
        best_score = float("inf")
        best_valid = 0.0
        second_score = float("inf")
        height, width = self.map_distance.shape
        ox, oy, oyaw = self.map_origin
        co = math.cos(oyaw)
        so = math.sin(oyaw)
        for start in range(0, xy_candidates.shape[0], self.score_chunk_size):
            chunk = xy_candidates[start:start + self.score_chunk_size]
            px = chunk[:, 0:1] + offsets[:, 0][None, :]
            py = chunk[:, 1:2] + offsets[:, 1][None, :]
            dx = px - ox
            dy = py - oy
            local_x = co * dx + so * dy
            local_y = -so * dx + co * dy
            cols = np.floor(local_x / self.map_resolution).astype(np.int32)
            rows = np.floor(local_y / self.map_resolution).astype(np.int32)
            valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
            valid_fraction = np.mean(valid, axis=1)
            safe_rows = np.clip(rows, 0, height - 1)
            safe_cols = np.clip(cols, 0, width - 1)
            dist = self.map_distance[safe_rows, safe_cols]
            dist = np.where(valid, dist, 1.0)
            scores = np.mean(np.minimum(dist, 1.0), axis=1) + (1.0 - valid_fraction) * 0.5
            order = np.argsort(scores)[:2]
            for local_i in order:
                score = float(scores[local_i])
                if score < best_score:
                    second_score = best_score
                    best_score = score
                    best_valid = float(valid_fraction[local_i])
                    best_index = start + int(local_i)
                elif score < second_score:
                    second_score = score
        return best_index, best_score, best_valid, second_score

    def search_global(self, points: np.ndarray) -> Tuple[Tuple[float, float, float], float, float, float]:
        xy_candidates = self.map_xy_from_cells(self.free_cells)
        yaws = np.arange(-math.pi, math.pi, self.global_yaw_step, dtype=np.float64)
        best_pose = (0.0, 0.0, 0.0)
        best_score = float("inf")
        best_valid = 0.0
        second_score = float("inf")
        for yaw in yaws:
            idx, score, valid, second = self.score_candidates(points, xy_candidates, float(yaw))
            if second < second_score:
                second_score = second
            if score < best_score and idx >= 0:
                second_score = min(second_score, best_score)
                best_score = score
                best_valid = valid
                best_pose = (
                    float(xy_candidates[idx, 0]),
                    float(xy_candidates[idx, 1]),
                    float(yaw),
                )
        return best_pose, best_score, best_valid, second_score

    def score_pose(self, points: np.ndarray, pose: Tuple[float, float, float]) -> Tuple[float, float]:
        candidate = np.asarray([[pose[0], pose[1]]], dtype=np.float32)
        _, score, valid, _ = self.score_candidates(points, candidate, pose[2])
        return score, valid

    def refine(self, points: np.ndarray, center: Tuple[float, float, float]) -> Tuple[Tuple[float, float, float], float, float]:
        best_pose = center
        best_score, best_valid = self.score_pose(points, center)
        xy_offsets = np.arange(
            -self.fine_xy_radius, self.fine_xy_radius + 1e-9,
            self.fine_xy_step, dtype=np.float64)
        yaw_offsets = np.arange(
            -self.fine_yaw_radius, self.fine_yaw_radius + 1e-9,
            self.fine_yaw_step, dtype=np.float64)
        for dyaw in yaw_offsets:
            yaw = wrap_angle(center[2] + float(dyaw))
            xy = []
            for dx in xy_offsets:
                for dy in xy_offsets:
                    xy.append((center[0] + float(dx), center[1] + float(dy)))
            xy_candidates = np.asarray(xy, dtype=np.float32)
            idx, score, valid, _ = self.score_candidates(points, xy_candidates, yaw)
            if idx >= 0 and score < best_score:
                best_score = score
                best_valid = valid
                best_pose = (float(xy_candidates[idx, 0]), float(xy_candidates[idx, 1]), yaw)
        return best_pose, best_score, best_valid

    def make_pose_msg(
        self, pose: Tuple[float, float, float], stamp, score: float, ambiguous: bool = False
    ) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        z, w = quat_from_yaw(pose[2])
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w
        sigma_xy = max(0.08, min(0.45, score * 0.9))
        sigma_yaw = math.radians(12.0)
        if ambiguous:
            # Long corridors can produce several similar scan-map matches.
            # Accept a stable repeated pose, but mark it less confident.
            sigma_xy = max(sigma_xy, 0.35)
            sigma_yaw = math.radians(25.0)
        msg.pose.covariance[0] = sigma_xy * sigma_xy
        msg.pose.covariance[7] = sigma_xy * sigma_xy
        msg.pose.covariance[35] = sigma_yaw * sigma_yaw
        return msg

    def accept_pose(
        self,
        msg: PoseWithCovarianceStamped,
        score: float,
        valid: float,
        margin: float,
        ambiguous: bool = False,
        mode: str = "",
    ):
        self.initialized = True
        self.last_pose = msg
        self.remaining_republishes = self.republish_count
        acceptance_mode = mode or ("ambiguous_stable" if ambiguous else "confident")
        self.get_logger().info(
            f"Global init seed published for scan-to-map confirmation: "
            f"pose=({msg.pose.pose.position.x:.3f}, "
            f"{msg.pose.pose.position.y:.3f}, {yaw_from_quat(msg.pose.pose.orientation):.3f}), "
            f"score={score:.3f}, valid={valid:.2f}, margin={margin:.3f}, "
            f"mode={acceptance_mode}")
        self.publish_status(
            f"seed_published awaiting_scan_map_confirmation "
            f"x={msg.pose.pose.position.x:.3f} y={msg.pose.pose.position.y:.3f} "
            f"yaw={yaw_from_quat(msg.pose.pose.orientation):.3f} score={score:.3f} "
            f"valid={valid:.2f} margin={margin:.3f} mode={acceptance_mode}")
        self.republish()

    def try_accept_stable_cluster(
        self,
        stamp,
        required: int,
        ambiguous: bool,
    ) -> bool:
        if len(self.successes) < required:
            return False

        best_cluster = []
        best_mean_score = float("inf")
        # Corridor scans often alternate between several symmetric hypotheses.
        # Look for a repeated local cluster, not only consecutive samples.
        for anchor_pose, _, _, _ in self.successes:
            cluster = []
            for sample in self.successes:
                pose = sample[0]
                if (
                    abs(pose[0] - anchor_pose[0]) <= self.stable_xy_tolerance and
                    abs(pose[1] - anchor_pose[1]) <= self.stable_xy_tolerance and
                    abs(wrap_angle(pose[2] - anchor_pose[2])) <= self.stable_yaw_tolerance
                ):
                    cluster.append(sample)
            if len(cluster) < required:
                continue
            recent_cluster = cluster[-required:]
            mean_score = float(np.mean([sample[1] for sample in recent_cluster]))
            if mean_score < best_mean_score:
                best_mean_score = mean_score
                best_cluster = recent_cluster

        if not best_cluster:
            return False

        xs = np.asarray([sample[0][0] for sample in best_cluster])
        ys = np.asarray([sample[0][1] for sample in best_cluster])
        yaws = np.asarray([sample[0][2] for sample in best_cluster])
        pose = (
            float(np.mean(xs)),
            float(np.mean(ys)),
            math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws))),
        )
        score = float(np.mean([sample[1] for sample in best_cluster]))
        valid = float(np.mean([sample[2] for sample in best_cluster]))
        margin = float(np.mean([sample[3] for sample in best_cluster]))
        self.accept_pose(
            self.make_pose_msg(pose, stamp, score, ambiguous),
            score, valid, margin, ambiguous,
            "stable_cluster" if ambiguous else "stable_confident_cluster")
        return True

    def republish(self):
        if self.last_pose is None or self.remaining_republishes <= 0:
            return
        # Repeated initial poses must carry a fresh timestamp; otherwise a
        # localization node may discard them as stale while it activates.
        self.last_pose.header.stamp = self.get_clock().now().to_msg()
        self.initialpose_pub.publish(self.last_pose)
        self.output_pose_pub.publish(self.last_pose)
        self.remaining_republishes -= 1

    def scan_callback(self, msg: LaserScan):
        if self.initialized or self.map_distance is None or self.free_cells is None:
            return
        self.scan_attempts += 1
        if self.scan_attempts > self.max_scan_attempts:
            self.publish_status("failed max_scan_attempts")
            return
        try:
            points = self.scan_points_in_base(msg)
        except TransformException as ex:
            self.get_logger().warn(f"Waiting for scan TF: {ex}")
            self.publish_status("waiting_for_scan_tf")
            return
        if points is None:
            self.publish_status("waiting_for_enough_scan_points")
            return

        global_pose, global_score, global_valid, second_score = self.search_global(points)
        pose, score, valid = self.refine(points, global_pose)
        margin = max(0.0, second_score - global_score)
        self.get_logger().info(
            f"Global init attempt {self.scan_attempts}: pose=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f}) "
            f"score={score:.3f}/{self.max_best_score:.3f} valid={valid:.2f}/{self.min_valid_fraction:.2f} margin={margin:.3f}/{self.min_score_margin:.3f}")
        self.publish_status(
            f"attempt={self.scan_attempts} score={score:.3f} valid={valid:.2f} margin={margin:.3f}")
        if score > self.max_best_score or valid < self.min_valid_fraction:
            return

        # Around the known home station, corridor symmetry can alternate
        # between nearby grid-cell solutions. A small cluster around home is
        # safer than waiting forever for identical pixel-level coordinates.
        if (self.enable_home_seed and
                math.hypot(pose[0] - self.home_x, pose[1] - self.home_y) <=
                self.home_seed_radius):
            self.home_successes.append((pose, score, valid, margin))
            keep = max(1, self.home_required_successes)
            self.home_successes = self.home_successes[-keep:]
            if len(self.home_successes) >= keep:
                home_yaws = np.asarray([sample[0][2] for sample in self.home_successes])
                yaw_span = max(
                    abs(wrap_angle(float(yaw) - float(home_yaws[0])))
                    for yaw in home_yaws)
                if yaw_span <= self.home_yaw_tolerance:
                    mean_yaw = math.atan2(
                        np.mean(np.sin(home_yaws)), np.mean(np.cos(home_yaws)))
                    mean_score = float(np.mean([sample[1] for sample in self.home_successes]))
                    mean_valid = float(np.mean([sample[2] for sample in self.home_successes]))
                    mean_margin = float(np.mean([sample[3] for sample in self.home_successes]))
                    home_pose = (self.home_x, self.home_y, mean_yaw)
                    self.accept_pose(
                        self.make_pose_msg(home_pose, msg.header.stamp, mean_score),
                        mean_score, mean_valid, mean_margin, False, "home_cluster")
                    return

        self.successes.append((pose, score, valid, margin))
        if len(self.successes) > self.max_success_history:
            self.successes = self.successes[-self.max_success_history:]

        current_confident = margin >= self.min_score_margin
        required = self.required_successes if current_confident else max(
            self.required_successes, self.ambiguous_required_successes)
        if self.try_accept_stable_cluster(
                msg.header.stamp, required, ambiguous=not current_confident):
            return
        if len(self.successes) < required:
            return
        recent = self.successes[-required:]
        confident = all(sample[3] >= self.min_score_margin for sample in recent)
        if not confident:
            required = max(self.required_successes, self.ambiguous_required_successes)
            if len(self.successes) < required:
                return
            recent = self.successes[-required:]
        xs = np.asarray([p[0][0] for p in recent])
        ys = np.asarray([p[0][1] for p in recent])
        yaws = np.asarray([p[0][2] for p in recent])
        yaw_span = max(abs(wrap_angle(float(yaw) - float(yaws[0]))) for yaw in yaws)
        if (np.ptp(xs) > self.stable_xy_tolerance or
                np.ptp(ys) > self.stable_xy_tolerance or
                yaw_span > self.stable_yaw_tolerance):
            return
        mean_yaw = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))
        pose = (
            float(np.mean(xs)),
            float(np.mean(ys)),
            mean_yaw,
        )
        score = float(np.mean([p[1] for p in recent]))
        valid = float(np.mean([p[2] for p in recent]))
        margin = float(np.mean([p[3] for p in recent]))
        ambiguous = not confident
        self.accept_pose(
            self.make_pose_msg(pose, msg.header.stamp, score, ambiguous),
            score, valid, margin, ambiguous)


def main():
    rclpy.init()
    node = GlobalScanInitializer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
