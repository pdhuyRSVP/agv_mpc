#!/usr/bin/env python3
import argparse
import math

import numpy as np


def rot_x(a):
    c = math.cos(a)
    s = math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c = math.cos(a)
    s = math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c = math.cos(a)
    s = math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(roll, pitch, yaw):
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def print_matrix_yaml(name, transform):
    flat = transform.reshape(-1)
    print(f"{name}: !!opencv-matrix")
    print("  rows: 4")
    print("  cols: 4")
    print("  dt: f")
    print("  data: [" + ", ".join(f"{v:.9f}" for v in flat) + "]")


def main():
    parser = argparse.ArgumentParser(
        description="Compute ORB-SLAM3 IMU.T_b_c1/Tbc from current AGV geometry.")
    parser.add_argument("--base-to-imu", nargs=3, type=float,
                        default=[0.0175, 0.014007, 0.1425])
    parser.add_argument("--base-to-camera", nargs=3, type=float,
                        default=[0.3875, -0.0030, 0.1425])
    parser.add_argument("--camera-rpy", nargs=3, type=float,
                        default=[-math.pi / 2.0, 0.0, -math.pi / 2.0],
                        help="Roll pitch yaw for base->camera optical frame.")
    args = parser.parse_args()

    p_base_imu = np.array(args.base_to_imu, dtype=float)
    p_base_cam = np.array(args.base_to_camera, dtype=float)
    r_base_imu = np.eye(3)
    r_base_cam = rpy_matrix(*args.camera_rpy)

    # ORB-SLAM3 IMU.T_b_c1 is T_body_camera: camera pose expressed in IMU/body frame.
    r_imu_cam = r_base_imu.T @ r_base_cam
    p_imu_cam = r_base_imu.T @ (p_base_cam - p_base_imu)

    tbc = np.eye(4)
    tbc[:3, :3] = r_imu_cam
    tbc[:3, 3] = p_imu_cam

    print("# Derived from current agv_mpc geometry:")
    print(f"# base_to_imu xyz:    {p_base_imu.tolist()}")
    print(f"# base_to_camera xyz: {p_base_cam.tolist()}")
    print(f"# camera rpy:         {args.camera_rpy}")
    print(f"# imu_to_camera xyz:  {p_imu_cam.tolist()}")
    print()
    print_matrix_yaml("IMU.T_b_c1", tbc)
    print()
    print_matrix_yaml("IMU.T_b_c", tbc)
    print()
    print_matrix_yaml("Tbc", tbc)


if __name__ == "__main__":
    main()
