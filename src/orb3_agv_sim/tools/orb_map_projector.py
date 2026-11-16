#!/usr/bin/env python3
import math
from copy import deepcopy

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_quaternion(q, x, y, z):
    # Quaternion vector rotation without external numpy dependency.
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


class OrbMapProjector(Node):
    def __init__(self):
        super().__init__("orb_map_projector")

        self.base_map_topic = self.declare_parameter("base_map_topic", "/map").value
        self.orb_points_topic = self.declare_parameter(
            "orb_points_topic", "/orb_rgbd/map_points").value
        self.orb_grid_topic = self.declare_parameter("orb_grid_topic", "/orb_2d_map").value
        self.combined_map_topic = self.declare_parameter("combined_map_topic", "/combined_map").value
        self.target_frame = self.declare_parameter("target_frame", "map").value
        self.min_z = float(self.declare_parameter("min_z", -0.20).value)
        self.max_z = float(self.declare_parameter("max_z", 2.00).value)
        self.occupied_value = int(self.declare_parameter("occupied_value", 70).value)
        self.unknown_orb_value = int(self.declare_parameter("unknown_orb_value", 45).value)
        self.inflation_radius = float(self.declare_parameter("inflation_radius", 0.05).value)
        self.publish_hz = float(self.declare_parameter("publish_hz", 1.0).value)
        self.max_points = int(self.declare_parameter("max_points", 12000).value)

        self.latest_map = None
        self.latest_cloud = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.orb_grid_pub = self.create_publisher(OccupancyGrid, self.orb_grid_topic, 2)
        self.combined_pub = self.create_publisher(OccupancyGrid, self.combined_map_topic, 2)

        self.create_subscription(OccupancyGrid, self.base_map_topic, self.map_cb, 2)
        self.create_subscription(PointCloud2, self.orb_points_topic, self.cloud_cb, 2)
        if self.publish_hz > 0.0:
            self.create_timer(1.0 / self.publish_hz, self.publish_maps)

        self.get_logger().info(
            f"Projecting {self.orb_points_topic} into {self.target_frame}; publishing "
            f"{self.orb_grid_topic} and {self.combined_map_topic}")
        self.get_logger().warn(
            "ORB map points are sparse visual features, not validated obstacles. Use combined_map for visualization/evaluation first.")

    def map_cb(self, msg):
        self.latest_map = msg

    def cloud_cb(self, msg):
        self.latest_cloud = msg

    def publish_maps(self):
        if self.latest_map is None or self.latest_cloud is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.latest_cloud.header.frame_id,
                rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting TF {self.target_frame} <- {self.latest_cloud.header.frame_id}: {exc}",
                throttle_duration_sec=2.0)
            return

        base = self.latest_map
        width = base.info.width
        height = base.info.height
        if width == 0 or height == 0 or base.info.resolution <= 0.0:
            return

        orb_grid = OccupancyGrid()
        orb_grid.header.stamp = self.get_clock().now().to_msg()
        orb_grid.header.frame_id = self.target_frame
        orb_grid.info = base.info
        orb_grid.data = [-1] * (width * height)

        combined = deepcopy(base)
        combined.header.stamp = orb_grid.header.stamp
        combined.header.frame_id = self.target_frame
        combined.data = list(base.data)

        resolution = base.info.resolution
        origin = base.info.origin.position
        origin_yaw = yaw_from_quaternion(base.info.origin.orientation)
        cos_o = math.cos(-origin_yaw)
        sin_o = math.sin(-origin_yaw)
        inflate_cells = max(0, int(math.ceil(self.inflation_radius / resolution)))

        marked = 0
        read_count = 0
        trans = transform.transform.translation
        rot = transform.transform.rotation

        for point in point_cloud2.read_points(
                self.latest_cloud, field_names=("x", "y", "z"), skip_nans=True):
            read_count += 1
            if self.max_points > 0 and read_count > self.max_points:
                break

            px, py, pz = float(point[0]), float(point[1]), float(point[2])
            rx, ry, rz = rotate_quaternion(rot, px, py, pz)
            mx = rx + trans.x
            my = ry + trans.y
            mz = rz + trans.z

            if mz < self.min_z or mz > self.max_z:
                continue

            dx = mx - origin.x
            dy = my - origin.y
            gx = cos_o * dx - sin_o * dy
            gy = sin_o * dx + cos_o * dy
            cx = int(math.floor(gx / resolution))
            cy = int(math.floor(gy / resolution))
            if cx < 0 or cy < 0 or cx >= width or cy >= height:
                continue

            for oy in range(-inflate_cells, inflate_cells + 1):
                for ox in range(-inflate_cells, inflate_cells + 1):
                    ix = cx + ox
                    iy = cy + oy
                    if ix < 0 or iy < 0 or ix >= width or iy >= height:
                        continue
                    if ox * ox + oy * oy > inflate_cells * inflate_cells:
                        continue
                    idx = iy * width + ix
                    orb_grid.data[idx] = self.occupied_value
                    if combined.data[idx] < 0:
                        combined.data[idx] = self.unknown_orb_value
                    else:
                        combined.data[idx] = max(combined.data[idx], self.occupied_value)
            marked += 1

        self.orb_grid_pub.publish(orb_grid)
        self.combined_pub.publish(combined)
        self.get_logger().info(
            f"ORB projected points: {marked}, input points scanned: {read_count}",
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = OrbMapProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
