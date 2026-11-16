#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    q = PoseStamped().pose.orientation
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def pose_to_se2(pose):
    return (
        float(pose.position.x),
        float(pose.position.y),
        yaw_from_quaternion(pose.orientation),
    )


def invert_se2(t):
    x, y, yaw = t
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        -(c * x + s * y),
        s * x - c * y,
        normalize_angle(-yaw),
    )


def compose_se2(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    c = math.cos(ayaw)
    s = math.sin(ayaw)
    return (
        ax + c * bx - s * by,
        ay + s * bx + c * by,
        normalize_angle(ayaw + byaw),
    )


def interpolate_se2(old, new, alpha):
    if old is None:
        return new
    ox, oy, oyaw = old
    nx, ny, nyaw = new
    return (
        ox + alpha * (nx - ox),
        oy + alpha * (ny - oy),
        normalize_angle(oyaw + alpha * normalize_angle(nyaw - oyaw)),
    )


def make_pose_stamped(stamp, frame_id, t):
    x, y, yaw = t
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0
    msg.pose.orientation = quaternion_from_yaw(yaw)
    return msg


class OrbDisplayCorrector(Node):
    def __init__(self):
        super().__init__("orb_display_corrector")

        self.orb_pose_topic = self.declare_parameter("orb_pose_topic", "/orb_rgbd/pose").value
        self.reference_pose_topic = self.declare_parameter(
            "reference_pose_topic", "/graph_fusion/pose").value
        self.corrected_pose_topic = self.declare_parameter(
            "corrected_pose_topic", "/orb_corrected/pose").value
        self.corrected_path_topic = self.declare_parameter(
            "corrected_path_topic", "/orb_corrected/path").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.orb_frame = self.declare_parameter("orb_frame", "orb_rgbd_map").value
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.continuous_correction = bool(
            self.declare_parameter("continuous_correction", True).value)
        self.correction_alpha = float(self.declare_parameter("correction_alpha", 0.05).value)
        self.max_path_length = int(self.declare_parameter("max_path_length", 2000).value)

        self.latest_reference = None
        self.T_map_orb = None
        self.path = Path()
        self.path.header.frame_id = self.map_frame

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, self.corrected_pose_topic, 20)
        self.path_pub = self.create_publisher(Path, self.corrected_path_topic, 10)

        self.create_subscription(
            PoseStamped,
            self.reference_pose_topic,
            self.reference_cb,
            50,
        )
        self.create_subscription(
            PoseStamped,
            self.orb_pose_topic,
            self.orb_cb,
            50,
        )

        self.get_logger().info(
            f"Correcting ORB display using {self.reference_pose_topic}; raw ORB: {self.orb_pose_topic}")
        self.get_logger().info(
            f"Publishing {self.map_frame}->{self.orb_frame} TF and corrected path {self.corrected_path_topic}")

    def reference_cb(self, msg):
        self.latest_reference = msg

    def orb_cb(self, msg):
        if self.latest_reference is None:
            self.get_logger().warn(
                f"Waiting reference pose on {self.reference_pose_topic}",
                throttle_duration_sec=2.0,
            )
            return

        T_orb_base = pose_to_se2(msg.pose)
        T_map_base_ref = pose_to_se2(self.latest_reference.pose)
        measured_T_map_orb = compose_se2(T_map_base_ref, invert_se2(T_orb_base))

        if self.T_map_orb is None or not self.continuous_correction:
            self.T_map_orb = measured_T_map_orb
            if self.T_map_orb == measured_T_map_orb:
                self.get_logger().info(
                    "ORB display correction initialized. ORB internal map is unchanged; only RViz/output frame is corrected.")
        else:
            alpha = min(1.0, max(0.0, self.correction_alpha))
            self.T_map_orb = interpolate_se2(self.T_map_orb, measured_T_map_orb, alpha)

        if self.publish_tf:
            self.publish_correction_tf(msg.header.stamp)

        corrected = compose_se2(self.T_map_orb, T_orb_base)
        pose_msg = make_pose_stamped(msg.header.stamp, self.map_frame, corrected)
        self.pose_pub.publish(pose_msg)

        self.path.header.stamp = pose_msg.header.stamp
        self.path.poses.append(pose_msg)
        if self.max_path_length > 0 and len(self.path.poses) > self.max_path_length:
            self.path.poses = self.path.poses[-self.max_path_length:]
        self.path_pub.publish(self.path)

    def publish_correction_tf(self, stamp):
        x, y, yaw = self.T_map_orb
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.orb_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = quaternion_from_yaw(yaw)
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = OrbDisplayCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
