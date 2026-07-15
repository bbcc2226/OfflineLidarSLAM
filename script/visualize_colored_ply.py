#!/usr/bin/env python3
"""Visualize a colored ASCII PLY point cloud with Open3D.

This helper loads the RGB stored in a PLY file written by
project_keyframes_to_image.py and renders the cloud with those colors.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Tuple

import numpy as np

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    raise SystemExit(
        "open3d is required. Install it with: pip install open3d"
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize a colored point cloud PLY with Open3D."
    )
    parser.add_argument("ply_path", type=Path, help="Path to a colored ASCII PLY file")
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.0,
        help="Optional voxel downsample size in meters. 0 disables downsampling.",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=2.0,
        help="Open3D point size used during visualization.",
    )
    parser.add_argument(
        "--window-name",
        type=str,
        default="Colored PLY Viewer",
        help="Open3D window title.",
    )
    parser.add_argument(
        "--background",
        choices=("dark", "light", "gray", "white", "black"),
        default="dark",
        help="Viewer background preset.",
    )
    return parser.parse_args()


def read_colored_ascii_ply(ply_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    if not ply_path.is_file():
        raise FileNotFoundError(f"PLY file not found: {ply_path}")

    vertex_count = None
    header_ended = False
    points: List[Tuple[float, float, float]] = []
    colors: List[Tuple[float, float, float]] = []

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
            if len(parts) < 6:
                continue

            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
            except ValueError:
                continue

            points.append((x, y, z))
            colors.append((r / 255.0, g / 255.0, b / 255.0))

            if vertex_count is not None and len(points) >= vertex_count:
                break

    if not points:
        raise ValueError(f"No colored vertices found in PLY file: {ply_path}")

    return np.asarray(points, dtype=np.float32), np.asarray(colors, dtype=np.float32)


def background_rgb(preset: str) -> Tuple[float, float, float]:
    mapping = {
        "dark": (0.08, 0.08, 0.08),
        "light": (0.92, 0.92, 0.92),
        "gray": (0.5, 0.5, 0.5),
        "white": (1.0, 1.0, 1.0),
        "black": (0.0, 0.0, 0.0),
    }
    return mapping[preset]


def main() -> int:
    args = parse_args()
    points, colors = read_colored_ascii_ply(args.ply_path)

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)

    if args.voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(args.voxel_size)

    print(
        f"Loaded {args.ply_path} with {len(cloud.points)} points and RGB colors"
    )

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=args.window_name)
    vis.add_geometry(cloud)
    render_option = vis.get_render_option()
    if render_option is not None:
        render_option.point_size = float(args.point_size)
        render_option.background_color = np.asarray(background_rgb(args.background), dtype=np.float32)

    vis.run()
    vis.destroy_window()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
