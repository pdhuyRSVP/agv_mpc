#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


class SlamToolboxTfPoseBridge(Node):
    """Publish slam_toolbox private TF as a map-frame pose factor.

    slam_toolbox runs with /tf remapped to a private topic. This node relays
    only robot-local TF into that private topic, never public map->odom, then
    composes:
      private map -> odom
      public  odom -> Base_footprint
    into /slam_toolbox/pose for GTSAM.
    """

    def __init__(self):
        super().__init__("slam_toolbox_tf_pose_bridge")
        self.tf_map_frame = self.declare_parameter(
            "tf_map_frame", "map").value
        self.output_frame = self.declare_parameter("output_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "Base_footprint").value
        self.pose_topic = self.declare_parameter("pose_topic", "/slam_toolbox/pose").value
        self.public_tf_topic = self.declare_parameter("public_tf_topic", "/tf").value
        self.private_tf_topic = self.declare_parameter(
            "private_tf_topic", "/slam_toolbox/tf").value
        self.relay_public_tf = bool(self.declare_parameter("relay_public_tf", True).value)
        allowed_pairs = self.declare_parameter(
            "allowed_public_tf_pairs",
            [
                "odom:Base_footprint",
                "Base_footprint:Base_Link",
                "Base_Link:Lidar",
                "Base_footprint:Lidar",
            ],
        ).value
        self.allowed_public_tf_pairs = {
            tuple(item.split(":", 1))
            for item in allowed_pairs
            if isinstance(item, str) and ":" in item
        }
        self.wait_for_posegraph_loaded = bool(
            self.declare_parameter("wait_for_posegraph_loaded", True).value)
        self.posegraph_loaded_topic = self.declare_parameter(
            "posegraph_loaded_topic", "/slam_toolbox/posegraph_loaded").value
        self.publish_hz = float(self.declare_parameter("publish_hz", 10.0).value)
        self.cov_x = float(self.declare_parameter("cov_x", 0.04).value)
        self.cov_y = float(self.declare_parameter("cov_y", 0.04).value)
        self.cov_yaw = float(self.declare_parameter("cov_yaw", 0.25).value)
        self.max_tf_time_delta_sec = float(
            self.declare_parameter("max_tf_time_delta_sec", 0.45).value)
        self.max_pose_age_sec = float(
            self.declare_parameter("max_pose_age_sec", 0.70).value)
        self.private_tf_future_tolerance_sec = float(
            self.declare_parameter("private_tf_future_tolerance_sec", 0.75).value)
        self.tf_lookup_timeout_sec = float(
            self.declare_parameter("tf_lookup_timeout_sec", 0.03).value)
        self.allow_latest_tf_fallback = bool(
            self.declare_parameter("allow_latest_tf_fallback", True).value)
        self.drop_on_tf_time_mismatch = bool(
            self.declare_parameter("drop_on_tf_time_mismatch", True).value)
        self.enable_graph_pose_gate = bool(
            self.declare_parameter("enable_graph_pose_gate", True).value)
        self.graph_pose_topic = self.declare_parameter(
            "graph_pose_topic", "/graph/pose").value
        self.max_graph_distance_sec = float(
            self.declare_parameter("max_graph_pose_age_sec", 1.0).value)
        self.max_graph_distance_xy = float(
            self.declare_parameter("max_graph_distance_xy", 1.0).value)
        self.max_graph_distance_yaw = float(
            self.declare_parameter("max_graph_distance_yaw", 3.2).value)
        self.latest_graph_pose = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.private_map_to_odom = None
        self.posegraph_loaded = not self.wait_for_posegraph_loaded
        self.private_tf_sub = self.create_subscription(
            TFMessage, self.private_tf_topic, self.private_tf_callback, 50)
        if self.relay_public_tf:
            self.private_tf_pub = self.create_publisher(TFMessage, self.private_tf_topic, 100)
            self.public_tf_sub = self.create_subscription(
                TFMessage, self.public_tf_topic, self.public_tf_callback, 100)
        else:
            self.private_tf_pub = None
            self.public_tf_sub = None
        loaded_qos = QoSProfile(depth=1)
        loaded_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        loaded_qos.reliability = ReliabilityPolicy.RELIABLE
        self.loaded_sub = self.create_subscription(
            Bool, self.posegraph_loaded_topic,
            self.posegraph_loaded_callback, loaded_qos)
        self.graph_pose_sub = self.create_subscription(
            PoseStamped, self.graph_pose_topic, self.graph_pose_callback, 20)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 20)
        self.add_on_set_parameters_callback(self.on_parameter_update)
        self.timer = self.create_timer(
            1.0 / max(self.publish_hz, 0.1), self.publish_pose)

        self.get_logger().info(
            f"Bridging private TF {self.tf_map_frame}->{self.odom_frame} "
            f"from {self.private_tf_topic} plus public {self.odom_frame}->{self.base_frame} "
            f"to {self.pose_topic} as frame {self.output_frame}; "
            f"relay_public_tf={self.relay_public_tf}, "
            f"allowed_pairs={sorted(self.allowed_public_tf_pairs)}, "
            f"wait_for_posegraph_loaded={self.wait_for_posegraph_loaded}, "
            f"graph_pose_gate={self.enable_graph_pose_gate}")

    def on_parameter_update(self, params):
        for param in params:
            if param.name == "max_tf_time_delta_sec":
                self.max_tf_time_delta_sec = float(param.value)
            elif param.name == "max_pose_age_sec":
                self.max_pose_age_sec = float(param.value)
            elif param.name == "private_tf_future_tolerance_sec":
                self.private_tf_future_tolerance_sec = float(param.value)
            elif param.name == "tf_lookup_timeout_sec":
                self.tf_lookup_timeout_sec = float(param.value)
            elif param.name == "drop_on_tf_time_mismatch":
                self.drop_on_tf_time_mismatch = bool(param.value)
            elif param.name == "allow_latest_tf_fallback":
                self.allow_latest_tf_fallback = bool(param.value)
        return SetParametersResult(successful=True)

    @staticmethod
    def yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def quat_from_yaw(yaw):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw * 0.5),
            w=math.cos(yaw * 0.5))

    def private_tf_callback(self, msg):
        for tf in msg.transforms:
            if (
                tf.header.frame_id == self.tf_map_frame and
                tf.child_frame_id == self.odom_frame
            ):
                if self.wait_for_posegraph_loaded and not self.posegraph_loaded:
                    continue
                self.private_map_to_odom = tf

    def posegraph_loaded_callback(self, msg):
        if not self.wait_for_posegraph_loaded:
            self.posegraph_loaded = True
            return
        loaded = bool(msg.data)
        if loaded != self.posegraph_loaded:
            self.private_map_to_odom = None
            self.get_logger().info(
                f"posegraph_loaded changed to {loaded}; clearing cached private TF")
        self.posegraph_loaded = loaded

    def graph_pose_callback(self, msg):
        self.latest_graph_pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            self.yaw_from_quat(msg.pose.orientation),
            Time.from_msg(msg.header.stamp).nanoseconds,
        )

    def public_tf_callback(self, msg):
        if self.private_tf_pub is None:
            return
        filtered = TFMessage()
        for tf in msg.transforms:
            parent = tf.header.frame_id
            child = tf.child_frame_id
            if parent == self.output_frame or child == self.output_frame:
                continue
            if (parent, child) in self.allowed_public_tf_pairs:
                filtered.transforms.append(tf)
        if filtered.transforms:
            self.private_tf_pub.publish(filtered)

    def publish_pose(self):
        if self.tf_map_frame != self.output_frame:
            self.get_logger().error(
                f"Refusing to publish {self.pose_topic}: private TF frame "
                f"{self.tf_map_frame} is not the output frame {self.output_frame}. "
                "Do not relabel slam_toolbox_map poses as map.",
                throttle_duration_sec=5.0)
            return
        if self.wait_for_posegraph_loaded and not self.posegraph_loaded:
            self.get_logger().warn(
                f"Waiting for {self.posegraph_loaded_topic}=true before publishing {self.pose_topic}",
                throttle_duration_sec=2.0)
            return
        if self.private_map_to_odom is None:
            self.get_logger().warn(
                f"Waiting for private slam_toolbox TF {self.tf_map_frame}->{self.odom_frame}",
                throttle_duration_sec=2.0)
            return

        map_to_odom = self.private_map_to_odom
        private_stamp = Time.from_msg(map_to_odom.header.stamp)
        now_ns = self.get_clock().now().nanoseconds
        private_stamp_ns = private_stamp.nanoseconds
        pose_age_sec = (now_ns - private_stamp_ns) * 1e-9
        private_tf_is_future = pose_age_sec < 0.0
        if (
            private_stamp_ns <= 0 or
            pose_age_sec > self.max_pose_age_sec or
            pose_age_sec < -self.private_tf_future_tolerance_sec
        ):
            self.get_logger().warn(
                f"Drop stale private TF: age={pose_age_sec:.3f}s "
                f"past_limit={self.max_pose_age_sec:.3f}s "
                f"future_limit={self.private_tf_future_tolerance_sec:.3f}s",
                throttle_duration_sec=1.0)
            return

        used_fallback = False
        lookup_time = Time() if private_tf_is_future else private_stamp
        lookup_reference_ns = now_ns if private_tf_is_future else private_stamp_ns
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                lookup_time,
                timeout=Duration(seconds=self.tf_lookup_timeout_sec))
        except TransformException as exact_exc:
            if not self.allow_latest_tf_fallback:
                self.get_logger().warn(
                    f"Cannot lookup public TF at private stamp: {exact_exc}",
                    throttle_duration_sec=1.0)
                return
            try:
                odom_to_base = self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_lookup_timeout_sec))
                used_fallback = True
            except TransformException as latest_exc:
                self.get_logger().warn(
                    f"Waiting for public TF {self.odom_frame}->{self.base_frame}: "
                    f"exact={exact_exc}; latest={latest_exc}",
                    throttle_duration_sec=1.0)
                return

        public_stamp_ns = Time.from_msg(odom_to_base.header.stamp).nanoseconds
        tf_delta_sec = abs(public_stamp_ns - lookup_reference_ns) * 1e-9
        if tf_delta_sec > self.max_tf_time_delta_sec:
            message = (
                f"TF time mismatch: private={private_stamp_ns * 1e-9:.6f} "
                f"public={public_stamp_ns * 1e-9:.6f} "
                f"reference={lookup_reference_ns * 1e-9:.6f} "
                f"delta={tf_delta_sec:.3f}s limit={self.max_tf_time_delta_sec:.3f}s "
                f"fallback={used_fallback} future_private={private_tf_is_future}")
            if self.drop_on_tf_time_mismatch:
                self.get_logger().warn(
                    f"Drop slam pose; {message}", throttle_duration_sec=1.0)
                return
            self.get_logger().warn(message, throttle_duration_sec=1.0)

        x_mo = map_to_odom.transform.translation.x
        y_mo = map_to_odom.transform.translation.y
        yaw_mo = self.yaw_from_quat(map_to_odom.transform.rotation)
        x_ob = odom_to_base.transform.translation.x
        y_ob = odom_to_base.transform.translation.y
        yaw_ob = self.yaw_from_quat(odom_to_base.transform.rotation)
        c = math.cos(yaw_mo)
        s = math.sin(yaw_mo)
        x_mb = x_mo + c * x_ob - s * y_ob
        y_mb = y_mo + s * x_ob + c * y_ob
        yaw_mb = math.atan2(math.sin(yaw_mo + yaw_ob), math.cos(yaw_mo + yaw_ob))
        if self.enable_graph_pose_gate and self.latest_graph_pose is not None:
            graph_x, graph_y, graph_yaw, graph_stamp_ns = self.latest_graph_pose
            graph_age_sec = (now_ns - graph_stamp_ns) * 1e-9
            if 0.0 <= graph_age_sec <= self.max_graph_distance_sec:
                dxy = math.hypot(x_mb - graph_x, y_mb - graph_y)
                dyaw = abs(math.atan2(math.sin(yaw_mb - graph_yaw), math.cos(yaw_mb - graph_yaw)))
                if dxy > self.max_graph_distance_xy or dyaw > self.max_graph_distance_yaw:
                    self.get_logger().warn(
                        f"Drop divergent slam pose: slam=({x_mb:.3f},{y_mb:.3f},{yaw_mb:.3f}) "
                        f"graph=({graph_x:.3f},{graph_y:.3f},{graph_yaw:.3f}) "
                        f"dxy={dxy:.3f}/{self.max_graph_distance_xy:.3f} "
                        f"dyaw={dyaw:.3f}/{self.max_graph_distance_yaw:.3f} "
                        f"graph_age={graph_age_sec:.3f}s",
                        throttle_duration_sec=1.0)
                    return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg() if private_tf_is_future else map_to_odom.header.stamp
        msg.header.frame_id = self.output_frame
        msg.pose.pose.position.x = x_mb
        msg.pose.pose.position.y = y_mb
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self.quat_from_yaw(yaw_mb)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = self.cov_x * self.cov_x
        msg.pose.covariance[7] = self.cov_y * self.cov_y
        msg.pose.covariance[14] = 1e-6
        msg.pose.covariance[21] = 1e-6
        msg.pose.covariance[28] = 1e-6
        msg.pose.covariance[35] = self.cov_yaw * self.cov_yaw
        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published synchronized slam pose: private_stamp="
            f"{private_stamp_ns * 1e-9:.6f} public_stamp="
            f"{public_stamp_ns * 1e-9:.6f} reference="
            f"{lookup_reference_ns * 1e-9:.6f} delta={tf_delta_sec:.3f}s "
            f"age={pose_age_sec:.3f}s fallback={used_fallback} "
            f"future_private={private_tf_is_future}",
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = SlamToolboxTfPoseBridge()
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
