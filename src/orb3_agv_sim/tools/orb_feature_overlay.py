#!/usr/bin/env python3
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class OrbFeatureOverlay(Node):
    def __init__(self):
        super().__init__("orb_feature_overlay")

        self.input_topic = self.declare_parameter(
            "input_topic", "/camera/color/image_raw").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/orb_features/image").value
        self.max_features = int(self.declare_parameter("max_features", 900).value)
        self.fast_threshold = int(self.declare_parameter("fast_threshold", 10).value)
        self.publish_hz = float(self.declare_parameter("publish_hz", 5.0).value)
        self.resize_scale = float(self.declare_parameter("resize_scale", 1.0).value)
        self.draw_rich_keypoints = bool(
            self.declare_parameter("draw_rich_keypoints", False).value)

        self.last_publish_time = 0.0
        self.orb = cv2.ORB_create(
            nfeatures=self.max_features,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            firstLevel=0,
            WTA_K=2,
            scoreType=cv2.ORB_HARRIS_SCORE,
            patchSize=31,
            fastThreshold=self.fast_threshold,
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, qos)
        self.pub = self.create_publisher(Image, self.output_topic, 1)

        self.get_logger().info(
            f"ORB feature overlay: {self.input_topic} -> {self.output_topic}, "
            f"features={self.max_features}, publish_hz={self.publish_hz}")

    def image_callback(self, msg):
        now = time.monotonic()
        if self.publish_hz > 0.0 and now - self.last_publish_time < 1.0 / self.publish_hz:
            return
        self.last_publish_time = now

        try:
            frame = self.imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")
            return

        if self.resize_scale > 0.0 and self.resize_scale != 1.0:
            frame = cv2.resize(
                frame,
                None,
                fx=self.resize_scale,
                fy=self.resize_scale,
                interpolation=cv2.INTER_AREA,
            )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints = self.orb.detect(gray, None)

        flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS if self.draw_rich_keypoints else 0
        output = cv2.drawKeypoints(
            frame,
            keypoints,
            None,
            color=(0, 255, 0),
            flags=flags,
        )
        cv2.putText(
            output,
            f"ORB keypoints: {len(keypoints)}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        out_msg = self.bgr_to_imgmsg(output)
        out_msg.header = msg.header
        self.pub.publish(out_msg)

    @staticmethod
    def imgmsg_to_bgr(msg):
        if msg.encoding not in ("bgr8", "rgb8", "mono8"):
            raise ValueError(f"unsupported encoding: {msg.encoding}")

        channels = 1 if msg.encoding == "mono8" else 3
        expected_step = msg.width * channels
        if msg.step < expected_step:
            raise ValueError(
                f"invalid step {msg.step}, expected at least {expected_step}")

        data = np.frombuffer(msg.data, dtype=np.uint8)
        image = data.reshape((msg.height, msg.step))[:, :expected_step]

        if channels == 1:
            mono = image.reshape((msg.height, msg.width))
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

        color = image.reshape((msg.height, msg.width, channels))
        if msg.encoding == "rgb8":
            return cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        return color.copy()

    @staticmethod
    def bgr_to_imgmsg(image):
        msg = Image()
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = image.shape[1] * 3
        msg.data = image.tobytes()
        return msg


def main():
    rclpy.init()
    node = OrbFeatureOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
