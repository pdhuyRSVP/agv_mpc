#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
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


class OrbAmclAligner(Node):
    def __init__(self):
        super().__init__("orb_amcl_aligner")

        self.orb_pose_topic = self.declare_parameter("orb_pose_topic", "/orb_rgbd/pose").value
        self.amcl_pose_topic = self.declare_parameter("amcl_pose_topic", "/amcl_pose").value
        self.output_pose_topic = self.declare_parameter(
            "output_pose_topic", "/orb_rgbd_aligned/pose").value
        self.output_pose_cov_topic = self.declare_parameter(
            "output_pose_cov_topic", "/orb_rgbd_aligned/pose_with_covariance").value
        self.output_path_topic = self.declare_parameter(
            "output_path_topic", "/orb_rgbd_aligned/path").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.orb_frame = self.declare_parameter("orb_frame", "orb_rgbd_map").value
        self.publish_map_to_orb_tf = bool(
            self.declare_parameter("publish_map_to_orb_tf", True).value)
        self.max_path_length = int(self.declare_parameter("max_path_length", 2000).value)

        self.cov_xy = float(self.declare_parameter("cov_xy", 0.04).value)
        self.cov_yaw = float(self.declare_parameter("cov_yaw", 0.04).value)

        self.latest_amcl = None
        self.T_map_orb = None
        self.path = Path()
        self.path.header.frame_id = self.map_frame

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 20)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.output_pose_cov_topic, 20)
        self.path_pub = self.create_publisher(Path, self.output_path_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.amcl_cb,
            20,
        )
        self.create_subscription(
            PoseStamped,
            self.orb_pose_topic,
            self.orb_cb,
            50,
        )

        self.get_logger().info(
            f"Aligning {self.orb_pose_topic} to {self.map_frame} using {self.amcl_pose_topic}")
        self.get_logger().info(
            f"Publishing aligned ORB pose: {self.output_pose_topic}, path: {self.output_path_topic}")
        if self.publish_map_to_orb_tf:
            self.get_logger().info(
                f"Publishing TF {self.map_frame} -> {self.orb_frame} for RViz ORB map overlay")

    def amcl_cb(self, msg):
        self.latest_amcl = msg

    def orb_cb(self, msg):
        if self.latest_amcl is None:
            self.get_logger().warn(
                "Waiting for AMCL pose before aligning ORB trajectory",
                throttle_duration_sec=2.0,
            )
            return

        T_orb_base = pose_to_se2(msg.pose)

        if self.T_map_orb is None:
            T_map_base_amcl = pose_to_se2(self.latest_amcl.pose.pose)
            self.T_map_orb = compose_se2(T_map_base_amcl, invert_se2(T_orb_base))
            self.get_logger().info(
                "ORB-AMCL alignment initialized. AMCL remains TF authority; ORB is aligned for compare/fusion topics.")

        if self.publish_map_to_orb_tf:
            self.publish_alignment_tf(msg.header.stamp)

        T_map_base_orb = compose_se2(self.T_map_orb, T_orb_base)
        pose_msg = make_pose_stamped(msg.header.stamp, self.map_frame, T_map_base_orb)

        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header = pose_msg.header
        pose_cov.pose.pose = pose_msg.pose
        pose_cov.pose.covariance[0] = self.cov_xy
        pose_cov.pose.covariance[7] = self.cov_xy
        pose_cov.pose.covariance[35] = self.cov_yaw

        self.pose_pub.publish(pose_msg)
        self.pose_cov_pub.publish(pose_cov)

        self.path.header.stamp = pose_msg.header.stamp
        self.path.poses.append(pose_msg)
        if self.max_path_length > 0 and len(self.path.poses) > self.max_path_length:
            self.path.poses = self.path.poses[-self.max_path_length:]
        self.path_pub.publish(self.path)

    def publish_alignment_tf(self, stamp):
        if self.T_map_orb is None:
            return

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
    node = OrbAmclAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
