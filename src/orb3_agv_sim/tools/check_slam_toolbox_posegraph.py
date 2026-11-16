#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class PosegraphChecker(Node):
    def __init__(self):
        super().__init__("check_slam_toolbox_posegraph")
        self.topic = self.declare_parameter(
            "topic", "/slam_toolbox/graph_visualization").value
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 5.0).value)
        self.message = None
        self.create_subscription(MarkerArray, self.topic, self.callback, 10)

    def callback(self, msg: MarkerArray):
        self.message = msg

    @staticmethod
    def marker_points(marker: Marker):
        return [(float(p.x), float(p.y), float(p.z)) for p in marker.points]

    @staticmethod
    def distance(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def summarize(self):
        if self.message is None:
            return None
        markers = self.message.markers
        point_count = 0
        node_markers = 0
        line_segments = []
        marker_summaries = []
        for marker in markers:
            pts = self.marker_points(marker)
            marker_summaries.append(
                f"ns={marker.ns or '-'} id={marker.id} type={marker.type} points={len(pts)}")
            if marker.type in (Marker.POINTS, Marker.SPHERE_LIST, Marker.CUBE_LIST):
                point_count += len(pts)
            if marker.type in (Marker.SPHERE, Marker.CUBE, Marker.ARROW):
                node_markers += 1
            if marker.type == Marker.LINE_LIST:
                for i in range(0, len(pts) - 1, 2):
                    line_segments.append((pts[i], pts[i + 1]))
            elif marker.type == Marker.LINE_STRIP:
                for i in range(0, len(pts) - 1):
                    line_segments.append((pts[i], pts[i + 1]))

        long_edges = [
            edge for edge in line_segments
            if self.distance(edge[0], edge[1]) > 0.75
        ]
        chain_edges = max(0, node_markers - 1)
        extra_edges = max(0, len(line_segments) - chain_edges)
        return {
            "markers": len(markers),
            "node_markers": node_markers,
            "point_count": point_count,
            "line_segments": len(line_segments),
            "chain_edges_estimate": chain_edges,
            "extra_edges_estimate": extra_edges,
            "long_edges": len(long_edges),
            "marker_summaries": marker_summaries[:20],
        }


def main():
    rclpy.init()
    node = PosegraphChecker()
    end_time = time.monotonic() + node.timeout_sec
    try:
        while rclpy.ok() and time.monotonic() < end_time and node.message is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        summary = node.summarize()
        if summary is None:
            print(f"No MarkerArray received on {node.topic}", file=sys.stderr)
            sys.exit(2)
        print(
            "posegraph_visualization "
            f"markers={summary['markers']} "
            f"node_markers={summary['node_markers']} "
            f"points={summary['point_count']} "
            f"line_segments={summary['line_segments']} "
            f"chain_edges_estimate={summary['chain_edges_estimate']} "
            f"extra_edges_estimate={summary['extra_edges_estimate']} "
            f"long_edges_gt_0p75m={summary['long_edges']}")
        for item in summary["marker_summaries"]:
            print(f"marker {item}")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
