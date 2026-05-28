#!/usr/bin/env python3
"""Publish the loop-closure submap and current scan(s) saved by VerifyLoopSubmap.

New files saved by VerifyLoopSubmap (requires save_loop_closure_debug_info: true):
  /tmp/loop_submap_kf<ID>.ply        — history submap (NDT target)       [blue]
  /tmp/loop_curr_raw_kf<ID>.ply      — raw current scan before NDT       [red]
  /tmp/loop_curr_aligned_kf<ID>.ply  — current scan after NDT alignment  [orange]
  /tmp/loop_init_guess_kf<ID>.txt    — initial guess pose (tx ty tz qw qx qy qz)

Usage:
  python3 submap_publisher.py <kf_id>                # submap + aligned scan
  python3 submap_publisher.py <kf_id> --show-raw     # adds raw pre-NDT scan (red)
  python3 submap_publisher.py <kf_id> --raw-only     # submap + raw scan only (skip aligned)
"""

import argparse
import struct
from pathlib import Path
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

DEFAULT_KF_ID = None  # set via --kf-id or override --submap/--curr directly

SUBMAP_COLOR  = np.array([26,  140, 242], dtype=np.uint8)  # blue   — history submap
CURR_COLOR    = np.array([242,  89,  38], dtype=np.uint8)  # orange — aligned current scan
RAW_COLOR     = np.array([220,  30,  30], dtype=np.uint8)  # red    — raw pre-NDT scan


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish loop-closure submap and aligned current scan to RViz2."
    )
    parser.add_argument("kf_id", type=int, nargs="?", default=None,
                        help="Keyframe ID to derive default PLY paths (/tmp/loop_submap_kfID.ply etc.)")
    parser.add_argument("--submap",    type=Path, default=None,
                        help="Path to submap PLY (overrides kf_id-derived path).")
    parser.add_argument("--curr",      type=Path, default=None,
                        help="Path to aligned current scan PLY (overrides kf_id-derived path).")
    parser.add_argument("--raw",       type=Path, default=None,
                        help="Path to raw (pre-NDT) current scan PLY (overrides kf_id-derived path).")
    parser.add_argument("--show-raw",  action="store_true",
                        help="Also publish the raw pre-NDT current scan (red) alongside the aligned one.")
    parser.add_argument("--raw-only",  action="store_true",
                        help="Show submap + raw pre-NDT scan; skip the post-alignment scan.")
    parser.add_argument("--print-init-guess", action="store_true",
                        help="Print the saved initial NDT guess pose and exit.")
    parser.add_argument("--topic",     default="/loop_closure_cloud",
                        help="PointCloud2 topic to publish.")
    parser.add_argument("--frame-id",  default="map",
                        help="TF frame_id used in the published PointCloud2 message.")
    parser.add_argument("--publish-rate", type=float, default=1.0,
                        help="Publish rate in Hz.")
    parser.add_argument("--max-points",   type=int,   default=300000,
                        help="Maximum total points after uniform downsampling. Non-positive disables.")
    return parser.parse_args()


def read_ascii_ply_vertices(ply_path: Path) -> np.ndarray:
    vertex_count = None
    header_ended = False
    points: List[Tuple[float, float, float]] = []

    with ply_path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not header_ended:
                if line.startswith("element vertex"):
                    vertex_count = int(line.split()[-1])
                elif line == "end_header":
                    header_ended = True
                continue
            if not line:
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                points.append((float(parts[0]), float(parts[1]), float(parts[2])))
            except ValueError:
                continue
            if vertex_count is not None and len(points) >= vertex_count:
                break

    if vertex_count is None:
        raise ValueError(f"Could not find vertex count in PLY header: {ply_path}")
    if not points:
        raise ValueError(f"No vertices found in PLY file: {ply_path}")
    return np.asarray(points, dtype=np.float32)


def uniform_downsample(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or points.shape[0] <= max_points:
        return points
    stride = int(np.ceil(points.shape[0] / max_points))
    return points[::stride]


def make_rgb_uint32(color: np.ndarray) -> int:
    return struct.unpack("I", struct.pack("BBBB", int(color[2]), int(color[1]), int(color[0]), 0))[0]


def colored_cloud(points: np.ndarray, color: np.ndarray) -> np.ndarray:
    rgb = np.full((points.shape[0], 1), make_rgb_uint32(color), dtype=np.uint32)
    return np.hstack((points.astype(np.float32, copy=False), rgb.view(np.float32)))


def resolve_paths(args: argparse.Namespace) -> argparse.Namespace:
    if args.kf_id is not None:
        if args.submap is None:
            args.submap = Path(f"/tmp/loop_submap_kf{args.kf_id}.ply")
        if args.curr is None:
            args.curr = Path(f"/tmp/loop_curr_aligned_kf{args.kf_id}.ply")
        if args.raw is None:
            args.raw = Path(f"/tmp/loop_curr_raw_kf{args.kf_id}.ply")
    if args.submap is None or args.curr is None:
        raise ValueError("Provide a keyframe ID (positional) or both --submap and --curr.")
    return args


def print_init_guess(kf_id: int) -> None:
    path = Path(f"/tmp/loop_init_guess_kf{kf_id}.txt")
    if not path.is_file():
        print(f"Init guess file not found: {path}")
        return
    values = path.read_text().split()
    if len(values) >= 7:
        tx, ty, tz, qw, qx, qy, qz = (float(v) for v in values[:7])
        print(f"Initial NDT guess for kf {kf_id}:")
        print(f"  translation : ({tx:.4f}, {ty:.4f}, {tz:.4f})")
        print(f"  quaternion  : w={qw:.6f}  x={qx:.6f}  y={qy:.6f}  z={qz:.6f}")
    else:
        print(f"Unexpected format in {path}: {path.read_text().strip()}")


def load_clouds(args: argparse.Namespace) -> np.ndarray:
    # Decide which clouds to show
    entries = [(args.submap, SUBMAP_COLOR, "submap")]
    if args.raw_only:
        if args.raw and args.raw.is_file():
            entries.append((args.raw, RAW_COLOR, "raw scan"))
        else:
            print(f"WARNING: raw scan not found at {args.raw}, falling back to aligned scan")
            entries.append((args.curr, CURR_COLOR, "aligned scan"))
    else:
        entries.append((args.curr, CURR_COLOR, "aligned scan"))
        if args.show_raw and args.raw and args.raw.is_file():
            entries.append((args.raw, RAW_COLOR, "raw scan"))
        elif args.show_raw:
            print(f"WARNING: raw scan not found at {args.raw}, --show-raw ignored")

    n = len(entries)
    budgets = [args.max_points * 2 // 3] + [args.max_points // (3 * max(n - 1, 1))] * (n - 1)

    clouds = []
    for (path, color, label), budget in zip(entries, budgets):
        if not path.is_file():
            raise FileNotFoundError(f"PLY not found for {label}: {path}")
        pts = read_ascii_ply_vertices(path)
        pts = uniform_downsample(pts, budget if args.max_points > 0 else 0)
        clouds.append(colored_cloud(pts, color))
        print(f"Loaded {label}: {pts.shape[0]} points from {path}")
    return np.vstack(clouds)


class SubmapPublisher(Node):
    def __init__(self, args: argparse.Namespace, cloud_points: np.ndarray):
        super().__init__("submap_publisher")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(PointCloud2, args.topic, qos)
        self.frame_id   = args.frame_id
        self.cloud_list = cloud_points.tolist()
        self.fields = [
            PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        period = 1.0 / max(args.publish_rate, 1e-6)
        self.timer = self.create_timer(period, self.publish_cloud)
        self.publish_cloud()

    def publish_cloud(self) -> None:
        header = Header()
        header.stamp   = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        msg = point_cloud2.create_cloud(header, self.fields, self.cloud_list)
        self.publisher.publish(msg)


def main() -> int:
    args = resolve_paths(parse_args())

    if args.print_init_guess:
        if args.kf_id is not None:
            print_init_guess(args.kf_id)
        else:
            print("--print-init-guess requires a keyframe ID.")
        return 0

    cloud_points = load_clouds(args)
    print(f"Publishing {cloud_points.shape[0]} points to {args.topic}")
    print("In RViz2, set the PointCloud2 display Color Transformer to RGB8.")

    rclpy.init()
    node = SubmapPublisher(args, cloud_points)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
