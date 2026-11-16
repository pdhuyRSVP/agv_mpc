#!/usr/bin/env python3
import argparse
import numpy as np
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def quat_to_matrix(x, y, z, w):
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x*x*s, y*y*s, z*z*s
    xy, xz, yz = x*y*s, x*z*s, y*z*s
    wx, wy, wz = w*x*s, w*y*s, w*z*s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ], dtype=float)


def print_yaml(name, T):
    flat = [T[i, j] for i in range(4) for j in range(4)]
    print(f"\n{name}: !!opencv-matrix")
    print("   rows: 4")
    print("   cols: 4")
    print("   dt: f")
    print("   data: [" + ", ".join(f"{v:.8f}" for v in flat) + "]")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--imu-frame', default='imu_link')
    ap.add_argument('--camera-frame', default='camera_link_optical')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('print_tbc_candidates')
    buf = Buffer()
    listener = TransformListener(buf, node)
    print('Waiting TF...')
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)

    try:
        tf = buf.lookup_transform(args.imu_frame, args.camera_frame, Time())
    except Exception as e:
        print('TF lookup failed:', e)
        print('Try: ros2 run tf2_tools view_frames')
        return

    q = tf.transform.rotation
    t = tf.transform.translation
    T_body_cam = np.eye(4)
    T_body_cam[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    T_body_cam[:3, 3] = [t.x, t.y, t.z]
    T_cam_body = np.linalg.inv(T_body_cam)

    print('\nCandidate for tbc_mode:=tf_body_camera')
    print(T_body_cam)
    print_yaml('Tbc', T_body_cam)
    print_yaml('IMU.T_b_c1', T_body_cam)

    print('\nCandidate for tbc_mode:=tf_camera_body / inverse')
    print(T_cam_body)
    print_yaml('Tbc', T_cam_body)
    print_yaml('IMU.T_b_c1', T_cam_body)

    print('\nBuilt-in manual_ros_optical used by this package:')
    T = np.eye(4)
    T[:3,:3] = np.array([[0,-1,0],[0,0,-1],[1,0,0]], float)
    print(T)
    print_yaml('Tbc', T)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
