#!/usr/bin/env python3
"""Inspect loop-closure debug clouds saved by VerifyLoopSubmap.

This script compares three states for a loop-closure candidate:
  1. history submap
  2. raw current scan transformed by the saved initial guess
  3. aligned current scan after registration

It opens a matplotlib figure with:
  - XY overlay before alignment
  - XY overlay after alignment
  - nearest-neighbor residual histograms before vs after alignment

It also prints summary metrics so the quality improvement is obvious.
"""

import argparse
from pathlib import Path
from typing import Tuple

import matplotlib
import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize loop-closure debug clouds and compare pre/post alignment quality."
    )
    parser.add_argument(
        "kf_id",
        type=int,
        help="Keyframe ID used to load /tmp/loop_* debug files.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=12000,
        help="Maximum points per cloud used in plots and metrics.",
    )
    parser.add_argument(
        "--hist-max-distance",
        type=float,
        default=5.0,
        help="Upper x-limit for residual histograms in meters.",
    )
    parser.add_argument(
        "--save-figure",
        type=Path,
        default=None,
        help="Optional output path for a PNG summary instead of only showing it interactively.",
    )
    return parser.parse_args()


def resolve_paths(args: argparse.Namespace) -> argparse.Namespace:
    args.submap = Path(f"/tmp/loop_submap_kf{args.kf_id}.ply")
    args.raw = Path(f"/tmp/loop_curr_raw_kf{args.kf_id}.ply")
    args.aligned = Path(f"/tmp/loop_curr_aligned_kf{args.kf_id}.ply")
    args.init = Path(f"/tmp/loop_init_guess_kf{args.kf_id}.txt")

    missing = [
        name for name, path in (
            ("submap", args.submap),
            ("raw", args.raw),
            ("aligned", args.aligned),
            ("init", args.init),
        )
        if path is None or not path.is_file()
    ]
    if missing:
        raise FileNotFoundError(
            "Missing required inputs: " + ", ".join(missing)
        )
    return args


def read_ascii_ply_vertices(ply_path: Path) -> np.ndarray:
    vertex_count = None
    header_ended = False
    points = []

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
            points.append((float(parts[0]), float(parts[1]), float(parts[2])))
            if vertex_count is not None and len(points) >= vertex_count:
                break

    if vertex_count is None:
        raise ValueError(f"Could not find vertex count in PLY header: {ply_path}")
    if not points:
        raise ValueError(f"No vertices found in PLY file: {ply_path}")
    return np.asarray(points, dtype=np.float64)


def uniform_downsample(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or points.shape[0] <= max_points:
        return points
    stride = int(np.ceil(points.shape[0] / max_points))
    return points[::stride]


def load_init_pose(path: Path) -> np.ndarray:
    values = [float(value) for value in path.read_text().split()]
    if len(values) < 7:
        raise ValueError(f"Unexpected init pose format in {path}")

    tx, ty, tz, qw, qx, qy, qz = values[:7]
    quat = np.array([qw, qx, qy, qz], dtype=np.float64)
    quat = quat / np.linalg.norm(quat)
    qw, qx, qy, qz = quat

    rotation = np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ])

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
    return transform


def apply_transform(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    rotated = points @ transform[:3, :3].T
    return rotated + transform[:3, 3]


def nearest_neighbor_distances(query: np.ndarray, reference: np.ndarray, chunk_size: int = 1024) -> np.ndarray:
    distances = np.empty(query.shape[0], dtype=np.float64)
    for start in range(0, query.shape[0], chunk_size):
        end = min(start + chunk_size, query.shape[0])
        chunk = query[start:end]
        diff = chunk[:, None, :] - reference[None, :, :]
        distances[start:end] = np.linalg.norm(diff, axis=2).min(axis=1)
    return distances


def summarize_distances(label: str, distances: np.ndarray) -> str:
    return (
        f"{label}: mean={distances.mean():.3f} m, median={np.median(distances):.3f} m, "
        f"p90={np.percentile(distances, 90):.3f} m, p95={np.percentile(distances, 95):.3f} m"
    )


def set_equal_xy_axes(axis: plt.Axes, clouds: Tuple[np.ndarray, ...]) -> None:
    stacked = np.vstack(clouds)
    mins = stacked[:, :2].min(axis=0)
    maxs = stacked[:, :2].max(axis=0)
    center = 0.5 * (mins + maxs)
    radius = 0.5 * np.max(maxs - mins)
    radius = max(radius, 1.0)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_aspect("equal", adjustable="box")


def backend_is_interactive() -> bool:
    return matplotlib.get_backend().lower() not in {"agg", "pdf", "ps", "svg", "cairo", "template"}


def default_output_path(kf_id: int) -> Path:
    return Path(__file__).resolve().parent / f"loop_closure_inspector_kf{kf_id}.png"


def main() -> int:
    args = resolve_paths(parse_args())

    submap = uniform_downsample(read_ascii_ply_vertices(args.submap), args.max_plot_points)
    raw_local = uniform_downsample(read_ascii_ply_vertices(args.raw), args.max_plot_points)
    aligned = uniform_downsample(read_ascii_ply_vertices(args.aligned), args.max_plot_points)
    init_transform = load_init_pose(args.init)
    raw_init = apply_transform(raw_local, init_transform)

    raw_dist = nearest_neighbor_distances(raw_init, submap)
    aligned_dist = nearest_neighbor_distances(aligned, submap)

    print(f"Loaded submap   : {submap.shape[0]} points from {args.submap}")
    print(f"Loaded raw      : {raw_local.shape[0]} points from {args.raw}")
    print(f"Loaded aligned  : {aligned.shape[0]} points from {args.aligned}")
    print(f"Loaded init pose: {args.init}")
    print(summarize_distances("Initial guess residual", raw_dist))
    print(summarize_distances("Aligned residual      ", aligned_dist))
    print(f"Mean improvement      : {raw_dist.mean() - aligned_dist.mean():.3f} m")
    print(f"Median improvement    : {np.median(raw_dist) - np.median(aligned_dist):.3f} m")

    fig, axes = plt.subplots(1, 3, figsize=(19, 6))

    axes[0].scatter(submap[:, 0], submap[:, 1], s=1, c="#1f77b4", alpha=0.35, label="Submap")
    axes[0].scatter(raw_init[:, 0], raw_init[:, 1], s=2, c="#d62728", alpha=0.55, label="Raw + init guess")
    axes[0].set_title("Before refinement")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.2)
    set_equal_xy_axes(axes[0], (submap, raw_init))

    axes[1].scatter(submap[:, 0], submap[:, 1], s=1, c="#1f77b4", alpha=0.35, label="Submap")
    axes[1].scatter(aligned[:, 0], aligned[:, 1], s=2, c="#ff7f0e", alpha=0.55, label="Aligned scan")
    axes[1].set_title("After refinement")
    axes[1].set_xlabel("x [m]")
    axes[1].set_ylabel("y [m]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.2)
    set_equal_xy_axes(axes[1], (submap, aligned))

    bins = np.linspace(0.0, args.hist_max_distance, 60)
    clipped_raw = np.clip(raw_dist, 0.0, args.hist_max_distance)
    clipped_aligned = np.clip(aligned_dist, 0.0, args.hist_max_distance)
    axes[2].hist(clipped_raw, bins=bins, alpha=0.55, color="#d62728", label="Raw + init guess")
    axes[2].hist(clipped_aligned, bins=bins, alpha=0.55, color="#ff7f0e", label="Aligned scan")
    axes[2].set_title("Nearest-neighbor residuals")
    axes[2].set_xlabel("distance to submap [m]")
    axes[2].set_ylabel("point count")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.2)

    fig.suptitle(
        f"Loop-closure inspection for kf {args.kf_id if args.kf_id is not None else 'custom'}",
        fontsize=14,
    )
    fig.tight_layout()

    output_path = args.save_figure
    if output_path is not None:
        fig.savefig(output_path, dpi=180, bbox_inches="tight")
        print(f"Saved figure to {output_path}")

    if backend_is_interactive():
        plt.show()
    else:
        if output_path is None:
            output_path = default_output_path(args.kf_id)
            fig.savefig(output_path, dpi=180, bbox_inches="tight")
            print(f"Non-interactive matplotlib backend detected; saved figure to {output_path}")
        plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())