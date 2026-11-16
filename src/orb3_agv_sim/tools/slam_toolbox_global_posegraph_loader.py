#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import DeserializePoseGraph
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SlamToolboxGlobalPosegraphLoader(Node):
    """Load and relocalize a serialized posegraph at a globally matched pose."""

    def __init__(self):
        super().__init__("slam_toolbox_global_posegraph_loader")
        self.pose_topic = self.declare_parameter(
            "pose_topic", "/global_scan_initializer/pose").value
        self.map_file_name = self.declare_parameter("map_file_name", "").value
        self.service_name = self.declare_parameter(
            "deserialize_service", "/slam_toolbox/deserialize_map").value
        self.loaded_topic = self.declare_parameter(
            "loaded_topic", "/slam_toolbox/posegraph_loaded").value
        self.slam_pose_topic = self.declare_parameter(
            "slam_pose_topic", "/slam_toolbox/pose").value
        self.graph_pose_topic = self.declare_parameter(
            "graph_pose_topic", "/graph/pose").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.trigger_topic = self.declare_parameter(
            "trigger_topic", "/global_scan_initializer/trigger").value
        self.relocalize_service = self.declare_parameter(
            "relocalize_service", "/relocalize_slam_toolbox").value
        self.relocalization_status_topic = self.declare_parameter(
            "relocalization_status_topic",
            "/slam_toolbox/relocalization_status").value
        self.match_type = int(self.declare_parameter(
            "match_type", DeserializePoseGraph.Request.LOCALIZE_AT_POSE).value)
        self.load_on_start = bool(
            self.declare_parameter("load_on_start", False).value)
        self.initial_x = float(self.declare_parameter("initial_x", 0.0).value)
        self.initial_y = float(self.declare_parameter("initial_y", 0.0).value)
        self.initial_yaw = float(self.declare_parameter("initial_yaw", 0.0).value)
        self.enable_relocalization = bool(
            self.declare_parameter("enable_relocalization", True).value)
        self.relocalize_distance_threshold = float(
            self.declare_parameter("relocalize_distance_threshold", 1.0).value)
        self.relocalize_yaw_threshold = float(
            self.declare_parameter("relocalize_yaw_threshold", 0.8).value)
        self.relocalize_reject_count_threshold = int(
            self.declare_parameter("relocalize_reject_count_threshold", 30).value)
        self.relocalize_cooldown_sec = float(
            self.declare_parameter("relocalize_cooldown_sec", 5.0).value)

        self.client = self.create_client(DeserializePoseGraph, self.service_name)
        loaded_qos = QoSProfile(depth=1)
        loaded_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        loaded_qos.reliability = ReliabilityPolicy.RELIABLE
        self.loaded_pub = self.create_publisher(Bool, self.loaded_topic, loaded_qos)
        self.trigger_pub = self.create_publisher(Bool, self.trigger_topic, 10)
        self.status_pub = self.create_publisher(
            String, self.relocalization_status_topic, 10)
        self.loaded = False
        self.awaiting_global_pose = True
        self.request_in_flight = False
        self.pending_pose = None
        self.latest_slam_pose = None
        self.latest_graph_pose = None
        self.latest_odom_pose = None
        self.map_info = None
        self.consecutive_divergent_poses = 0
        self.last_relocalize_time = -1e9
        self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.slam_pose_topic,
            self.slam_pose_callback, 20)
        self.create_subscription(
            PoseStamped, self.graph_pose_topic, self.graph_pose_callback, 20)
        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 20)
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_service(
            Trigger, self.relocalize_service, self.relocalize_service_callback)
        self.timer = self.create_timer(0.25, self.try_load)
        self.publish_loaded(False)
        if self.load_on_start:
            self.pending_pose = self.make_initial_pose()
        self.get_logger().info(
            f"Waiting for global pose on {self.pose_topic} to load posegraph "
            f"{self.map_file_name} via {self.service_name}; "
            f"loaded status on {self.loaded_topic}; "
            f"manual relocalization service={self.relocalize_service}; "
            f"load_on_start={self.load_on_start}")

    def make_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        half = 0.5 * self.initial_yaw
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        return msg

    def publish_loaded(self, value: bool):
        msg = Bool()
        msg.data = bool(value)
        self.loaded_pub.publish(msg)

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if self.loaded and not self.awaiting_global_pose:
            return
        self.pending_pose = msg
        self.try_load()

    def try_load(self):
        if (
            self.pending_pose is None or
            self.request_in_flight or
            not self.awaiting_global_pose
        ):
            return
        if not self.map_file_name:
            self.get_logger().error("map_file_name is empty; cannot load slam_toolbox posegraph")
            self.publish_loaded(False)
            return
        if not self.client.service_is_ready():
            self.client.wait_for_service(timeout_sec=0.0)
            return

        pose = self.pending_pose.pose.pose
        request = DeserializePoseGraph.Request()
        request.filename = self.map_file_name
        request.match_type = self.match_type
        request.initial_pose.x = float(pose.position.x)
        request.initial_pose.y = float(pose.position.y)
        request.initial_pose.theta = yaw_from_quat(pose.orientation)

        self.request_in_flight = True
        self.publish_loaded(False)
        self.get_logger().info(
            "Loading slam_toolbox posegraph at global pose "
            f"x={request.initial_pose.x:.3f}, y={request.initial_pose.y:.3f}, "
            f"yaw={request.initial_pose.theta:.3f}, match_type={request.match_type}")
        future = self.client.call_async(request)
        future.add_done_callback(self.load_done)

    def load_done(self, future):
        self.request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.loaded = False
            self.publish_loaded(False)
            self.get_logger().error(f"Failed to call deserialize_map: {exc}")
            return
        result = getattr(response, "result", None)
        if result is None:
            self.get_logger().warn(
                "slam_toolbox deserialize_map returned an empty response; "
                "assuming the request was accepted because some slam_toolbox "
                "versions do not populate result in localization mode")
            result = 0
        if result != 0:
            self.loaded = False
            self.awaiting_global_pose = True
            self.pending_pose = None
            self.publish_loaded(False)
            status = (
                f"posegraph_load_failed result={result} "
                f"map_file_name={self.map_file_name}"
            )
            self.publish_status(status)
            self.get_logger().error(
                "slam_toolbox deserialize_map failed: "
                f"result={result}; posegraph_loaded=False; "
                f"map_file_name={self.map_file_name}")
            return
        self.loaded = True
        self.awaiting_global_pose = False
        self.pending_pose = None
        self.consecutive_divergent_poses = 0
        self.publish_loaded(True)
        self.publish_status(f"posegraph_loaded result={result}")
        self.get_logger().info(
            f"slam_toolbox deserialize_map response result={result}; posegraph_loaded=True")

    @staticmethod
    def pose2d(pose):
        return (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quat(pose.orientation),
        )

    @staticmethod
    def angle_error(a: float, b: float) -> float:
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    def graph_pose_callback(self, msg: PoseStamped):
        self.latest_graph_pose = self.pose2d(msg.pose)

    def odom_callback(self, msg: Odometry):
        self.latest_odom_pose = self.pose2d(msg.pose.pose)

    def map_callback(self, msg: OccupancyGrid):
        origin = msg.info.origin
        self.map_info = (
            float(origin.position.x),
            float(origin.position.y),
            yaw_from_quat(origin.orientation),
            float(msg.info.resolution),
            int(msg.info.width),
            int(msg.info.height),
        )

    def pose_inside_map(self, pose) -> bool:
        if self.map_info is None:
            return True
        ox, oy, oyaw, resolution, width, height = self.map_info
        dx = pose[0] - ox
        dy = pose[1] - oy
        c = math.cos(oyaw)
        s = math.sin(oyaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        return (
            0.0 <= local_x <= width * resolution and
            0.0 <= local_y <= height * resolution
        )

    def slam_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.latest_slam_pose = self.pose2d(msg.pose.pose)
        if (
            not self.enable_relocalization or
            not self.loaded or
            self.awaiting_global_pose or
            self.latest_graph_pose is None
        ):
            return

        slam = self.latest_slam_pose
        graph = self.latest_graph_pose
        distance = math.hypot(slam[0] - graph[0], slam[1] - graph[1])
        yaw_error = self.angle_error(slam[2], graph[2])
        outside_map = not self.pose_inside_map(slam)
        divergent = (
            outside_map or
            distance > self.relocalize_distance_threshold or
            yaw_error > self.relocalize_yaw_threshold
        )
        if not divergent:
            self.consecutive_divergent_poses = 0
            return

        self.consecutive_divergent_poses += 1
        reason = (
            f"slam_graph_divergence "
            f"slam=({slam[0]:.3f},{slam[1]:.3f},{slam[2]:.3f}) "
            f"graph=({graph[0]:.3f},{graph[1]:.3f},{graph[2]:.3f}) "
            f"dxy={distance:.3f}/"
            f"{self.relocalize_distance_threshold:.3f} "
            f"dyaw={yaw_error:.3f}/{self.relocalize_yaw_threshold:.3f} "
            f"outside_map={outside_map} count={self.consecutive_divergent_poses}/"
            f"{self.relocalize_reject_count_threshold}"
        )
        self.get_logger().warn(reason, throttle_duration_sec=1.0)
        if (
            outside_map or
            self.consecutive_divergent_poses >=
            max(1, self.relocalize_reject_count_threshold)
        ):
            self.begin_relocalization(reason)

    def begin_relocalization(self, reason: str, force: bool = False) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.awaiting_global_pose or self.request_in_flight:
            return False
        if not force and now - self.last_relocalize_time < self.relocalize_cooldown_sec:
            return False

        self.last_relocalize_time = now
        self.loaded = False
        self.awaiting_global_pose = True
        self.pending_pose = None
        self.consecutive_divergent_poses = 0
        self.publish_loaded(False)
        trigger = Bool()
        trigger.data = True
        self.trigger_pub.publish(trigger)
        odom_text = "none" if self.latest_odom_pose is None else (
            f"({self.latest_odom_pose[0]:.3f},{self.latest_odom_pose[1]:.3f},"
            f"{self.latest_odom_pose[2]:.3f})")
        status = f"relocalization_triggered reason={reason} odom={odom_text}"
        self.publish_status(status)
        self.get_logger().error(status)
        return True

    def relocalize_service_callback(self, _request, response):
        started = self.begin_relocalization("manual_service", force=True)
        response.success = started
        response.message = (
            "Global scan relocalization started"
            if started else
            "Relocalization is already in progress")
        return response


def main():
    rclpy.init()
    node = SlamToolboxGlobalPosegraphLoader()
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
