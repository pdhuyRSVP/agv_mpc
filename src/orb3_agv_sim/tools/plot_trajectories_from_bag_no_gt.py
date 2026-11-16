#!/usr/bin/env python3
import argparse
from pathlib import Path
import csv
import math

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = {
    '/orbslam3/pose': 'ORB-SLAM3 IMU',
    '/odometry/filtered': 'EKF',
    '/amcl_pose': 'AMCL',
}


def yaw_from_q(q):
    siny = 2.0 * (q.w*q.z + q.x*q.y)
    cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    return math.atan2(siny, cosy)


def pose_from_msg(msg):
    # PoseStamped
    if hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):
        p = msg.pose.position
        q = msg.pose.orientation
        return p.x, p.y, yaw_from_q(q)
    # Odometry
    if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose') and hasattr(msg.pose.pose, 'position'):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return p.x, p.y, yaw_from_q(q)
    # PoseWithCovarianceStamped
    if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose') and hasattr(msg.pose.pose, 'position'):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return p.x, p.y, yaw_from_q(q)
    return None


def read_bag(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {name: get_message(typ) for name, typ in topic_types.items() if name in TOPICS}
    data = {name: [] for name in TOPICS}
    while reader.has_next():
        topic, raw, t = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(raw, msg_types[topic])
        pose = pose_from_msg(msg)
        if pose is not None:
            data[topic].append((t * 1e-9, *pose))
    return data


def start_align(arr, ref0=None):
    if len(arr) == 0:
        return arr
    arr = np.asarray(arr, dtype=float)
    x0, y0, yaw0 = arr[0,1], arr[0,2], arr[0,3]
    if ref0 is None:
        rx, ry, ryaw = 0.0, 0.0, 0.0
    else:
        rx, ry, ryaw = ref0
    c, s = math.cos(ryaw-yaw0), math.sin(ryaw-yaw0)
    xy = arr[:,1:3] - np.array([x0,y0])
    R = np.array([[c,-s],[s,c]])
    out = arr.copy()
    out[:,1:3] = xy @ R.T + np.array([rx,ry])
    out[:,3] = arr[:,3] + (ryaw-yaw0)
    return out


def plot(data, out_dir, aligned=False):
    plt.figure(figsize=(9,7))
    ref0 = None
    if aligned and len(data.get('/amcl_pose', [])) > 0:
        a = np.asarray(data['/amcl_pose'], dtype=float)
        ref0 = (a[0,1], a[0,2], a[0,3])
    for topic, label in TOPICS.items():
        arr = np.asarray(data.get(topic, []), dtype=float)
        if arr.size == 0:
            continue
        if aligned:
            arr = start_align(arr, ref0)
        plt.plot(arr[:,1], arr[:,2], label=f'{label} ({len(arr)})')
    plt.axis('equal')
    plt.grid(True)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend()
    plt.title('Trajectories' + (' start-aligned' if aligned else ' raw'))
    fn = out_dir / ('trajectories_start_aligned_no_gt.png' if aligned else 'trajectories_raw_no_gt.png')
    plt.savefig(fn, dpi=160, bbox_inches='tight')
    print('wrote', fn)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--output-dir', required=True)
    args = ap.parse_args()
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    data = read_bag(Path(args.bag))
    plot(data, out_dir, aligned=False)
    plot(data, out_dir, aligned=True)
    with open(out_dir / 'trajectory_counts.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['topic','label','samples'])
        for topic, label in TOPICS.items():
            w.writerow([topic, label, len(data.get(topic, []))])
    print('wrote', out_dir / 'trajectory_counts.csv')

if __name__ == '__main__':
    main()
