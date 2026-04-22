#!/usr/bin/env python3
"""Publish the loop-closure submap and aligned current scan saved by VerifyLoopSubmap."""

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

SUBMAP_COLOR = np.array([26, 140, 242], dtype=np.uint8)   # blue  — history submap
CURR_COLOR   = np.array([242, 89,  38], dtype=np.uint8)   # orange — aligned current scan


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
    if args.submap is None or args.curr is None:
        raise ValueError("Provide a keyframe ID (positional) or both --submap and --curr.")
    return args


def load_clouds(args: argparse.Namespace) -> np.ndarray:
    clouds = []
    for path, color, label, budget in (
        (args.submap, SUBMAP_COLOR, "submap",       args.max_points * 2 // 3),
        (args.curr,   CURR_COLOR,   "aligned scan",  args.max_points // 3),
    ):
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
