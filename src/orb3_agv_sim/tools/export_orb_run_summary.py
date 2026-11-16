#!/usr/bin/env python3
import argparse
import csv
import math
import os
from pathlib import Path

import cv2
import numpy as np


def read_xyz(path):
    points = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                points.append((float(row["x"]), float(row["y"]), float(row["z"])))
            except (KeyError, TypeError, ValueError):
                continue
    return np.asarray(points, dtype=np.float32)


def read_last_metrics(path):
    last = {}
    if not path.exists():
        return last
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            last = row
    return last


def render_points(points, out_path, image_size=1200):
    canvas = np.full((image_size, image_size, 3), 255, dtype=np.uint8)
    if points.size == 0:
        cv2.putText(canvas, "No map points", (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.imwrite(str(out_path), canvas)
        return

    xy = points[:, :2]
    z = points[:, 2]
    lo = xy.min(axis=0)
    hi = xy.max(axis=0)
    span = np.maximum(hi - lo, 1e-3)
    margin = 60
    scale = min((image_size - 2 * margin) / span[0], (image_size - 2 * margin) / span[1])
    pix = (xy - lo) * scale + margin
    pix[:, 1] = image_size - pix[:, 1]

    z_min = float(np.min(z))
    z_max = float(np.max(z))
    z_span = max(z_max - z_min, 1e-6)
    colors = ((z - z_min) / z_span * 255.0).astype(np.uint8)

    for (px, py), c in zip(pix, colors):
        color = cv2.applyColorMap(np.asarray([[c]], dtype=np.uint8), cv2.COLORMAP_TURBO)[0, 0].tolist()
        cv2.circle(canvas, (int(px), int(py)), 2, color, -1, lineType=cv2.LINE_AA)

    cv2.rectangle(canvas, (margin, margin), (image_size - margin, image_size - margin), (40, 40, 40), 1)
    lines = [
        f"ORB map points: {len(points)}",
        f"x: {lo[0]:.2f} .. {hi[0]:.2f} m",
        f"y: {lo[1]:.2f} .. {hi[1]:.2f} m",
        f"z: {z_min:.2f} .. {z_max:.2f} m",
    ]
    for i, text in enumerate(lines):
        cv2.putText(canvas, text, (30, 35 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (20, 20, 20), 2)
    cv2.imwrite(str(out_path), canvas)


def find_latest_orb_dir(base):
    base = Path(base).expanduser()
    if (base / "map_points.csv").exists():
        return base
    children = [p for p in base.iterdir() if p.is_dir() and (p / "map_points.csv").exists()]
    if not children:
        raise FileNotFoundError(f"No ORB output directory with map_points.csv under {base}")
    return max(children, key=lambda p: p.stat().st_mtime)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("orb_output_dir", help="ORB output dir or parent dir containing timestamp run dirs")
    parser.add_argument("--output-dir", default="", help="Directory for final_map_points.png and eval_summary.txt")
    args = parser.parse_args()

    orb_dir = find_latest_orb_dir(args.orb_output_dir)
    out_dir = Path(args.output_dir).expanduser() if args.output_dir else orb_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    points = read_xyz(orb_dir / "map_points.csv")
    render_points(points, out_dir / "final_map_points.png")
    metrics = read_last_metrics(orb_dir / "tracking_metrics.csv")

    with open(out_dir / "eval_summary.txt", "w", encoding="utf-8") as f:
        f.write(f"orb_output_dir: {orb_dir}\n")
        f.write(f"map_points_csv_count: {len(points)}\n")
        if points.size:
            f.write(f"map_points_x_min: {float(points[:, 0].min()):.6f}\n")
            f.write(f"map_points_x_max: {float(points[:, 0].max()):.6f}\n")
            f.write(f"map_points_y_min: {float(points[:, 1].min()):.6f}\n")
            f.write(f"map_points_y_max: {float(points[:, 1].max()):.6f}\n")
            f.write(f"map_points_z_min: {float(points[:, 2].min()):.6f}\n")
            f.write(f"map_points_z_max: {float(points[:, 2].max()):.6f}\n")
        for key in ("keyframes", "map_points", "tracking_state_name", "tracked_keypoints", "tracked_mappoints"):
            if key in metrics:
                f.write(f"last_{key}: {metrics[key]}\n")

    print(f"ORB run: {orb_dir}")
    print(f"Wrote: {out_dir / 'final_map_points.png'}")
    print(f"Wrote: {out_dir / 'eval_summary.txt'}")


if __name__ == "__main__":
    main()
