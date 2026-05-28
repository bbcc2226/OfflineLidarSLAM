#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np


WGS84_A = 6378137.0
WGS84_F = 0.0033528106647474805
WGS84_B = 6356752.314245
WGS84_E2 = 0.00669437999014


def geo_to_ecef(lat_deg: float, lon_deg: float, alt: float) -> np.ndarray:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) * math.sin(lat))

    x = (radius + alt) * math.cos(lat) * math.cos(lon)
    y = (radius + alt) * math.cos(lat) * math.sin(lon)
    z = (radius * (1.0 - WGS84_E2) + alt) * math.sin(lat)
    return np.array([x, y, z], dtype=np.float64)


def ecef_to_enu_rotation(lat0_deg: float, lon0_deg: float) -> np.ndarray:
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)

    return np.array(
        [
            [-math.sin(lon0), math.cos(lon0), 0.0],
            [
                -math.sin(lat0) * math.cos(lon0),
                -math.sin(lat0) * math.sin(lon0),
                math.cos(lat0),
            ],
            [
                math.cos(lat0) * math.cos(lon0),
                math.cos(lat0) * math.sin(lon0),
                math.sin(lat0),
            ],
        ],
        dtype=np.float64,
    )


def resolve_data_dir(oxts_path: Path) -> Path:
    if oxts_path.is_dir() and oxts_path.name == "data":
        return oxts_path

    data_dir = oxts_path / "data"
    if data_dir.is_dir():
        return data_dir

    raise FileNotFoundError(f"Could not find OXTS data directory under: {oxts_path}")


def load_oxts_positions(data_dir: Path) -> np.ndarray:
    txt_files = sorted(data_dir.glob("*.txt"))
    if not txt_files:
        raise FileNotFoundError(f"No OXTS txt files found in: {data_dir}")

    gps_positions = []
    for txt_file in txt_files:
        values = np.loadtxt(txt_file, dtype=np.float64)
        if values.shape[0] < 3:
            raise ValueError(f"Invalid OXTS row in {txt_file}: expected at least 3 values")
        gps_positions.append(values[:3])

    return np.asarray(gps_positions, dtype=np.float64)


def parse_timestamp_ns(text: str) -> int:
    sec_str, nano_str = text.split(".")
    dt = datetime.strptime(sec_str, "%Y-%m-%d %H:%M:%S")
    return int(dt.timestamp() * 1e9) + int(nano_str.ljust(9, "0"))


def load_oxts_timestamps_ns(oxts_path: Path) -> np.ndarray:
    timestamps_path = oxts_path / "timestamps.txt"
    if not timestamps_path.is_file():
        raise FileNotFoundError(f"Could not find timestamps.txt under: {oxts_path}")

    lines = [line.strip() for line in timestamps_path.read_text().splitlines() if line.strip()]
    return np.asarray([parse_timestamp_ns(line) for line in lines], dtype=np.int64)


def gps_to_enu(gps_positions: np.ndarray) -> np.ndarray:
    ref = gps_positions[0]
    ref_ecef = geo_to_ecef(ref[0], ref[1], ref[2])
    rotation = ecef_to_enu_rotation(ref[0], ref[1])

    enu_points = []
    for lat, lon, alt in gps_positions:
        ecef = geo_to_ecef(lat, lon, alt)
        enu_points.append(rotation @ (ecef - ref_ecef))

    return np.asarray(enu_points, dtype=np.float64)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read KITTI OXTS txt files and draw the GPS trajectory in the ENU frame."
    )
    parser.add_argument(
        "oxts_path",
        nargs="?",
        default="/root/ros2_ws/data/2011_10_03_drive_0027/oxts",
        help="Path to the OXTS directory or directly to its data/ subdirectory.",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=10,
        help="Plot every Nth frame to reduce clutter.",
    )
    parser.add_argument(
        "--save",
        type=Path,
        help="Optional output image path. If omitted, an interactive window is shown.",
    )
    parser.add_argument(
        "--title",
        default="OXTS XY trajectory in ENU frame",
        help="Plot title.",
    )
    parser.add_argument(
        "--hide-gap-labels",
        action="store_true",
        help="Disable distance labels between consecutive sampled points.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=5000,
        help="Use only the first N OXTS samples before applying stride.",
    )
    return parser


def annotate_gaps(ax: plt.Axes, enu_points: np.ndarray, timestamps_ns: np.ndarray) -> None:
    if enu_points.shape[0] < 2:
        return

    for idx in range(1, enu_points.shape[0]):
        prev_pt = enu_points[idx - 1]
        curr_pt = enu_points[idx]
        gap = np.linalg.norm(curr_pt[:2] - prev_pt[:2])
        mid_pt = 0.5 * (prev_pt + curr_pt)
        dt_sec = (timestamps_ns[idx] - timestamps_ns[idx - 1]) / 1e9

        ax.plot(
            [prev_pt[0], curr_pt[0]],
            [prev_pt[1], curr_pt[1]],
            linestyle="--",
            linewidth=0.8,
            color="gray",
            alpha=0.6,
        )

        label = f"{gap:.1f} m"
        if gap > 5.0:
            label += f"\n{dt_sec:.3f} s"

        ax.text(
            mid_pt[0],
            mid_pt[1],
            label,
            fontsize=7,
            color="black",
            ha="center",
            va="bottom",
            bbox={"boxstyle": "round,pad=0.15", "facecolor": "white", "alpha": 0.7, "edgecolor": "none"},
        )


def main() -> None:
    args = build_parser().parse_args()

    if args.stride <= 0:
        raise ValueError("--stride must be a positive integer")
    if args.max_points <= 0:
        raise ValueError("--max-points must be a positive integer")

    data_dir = resolve_data_dir(Path(args.oxts_path))
    gps_positions = load_oxts_positions(data_dir)
    timestamps_ns = load_oxts_timestamps_ns(data_dir.parent)
    enu_points = gps_to_enu(gps_positions)
    enu_points = enu_points[: args.max_points]
    timestamps_ns = timestamps_ns[: args.max_points]
    enu_points = enu_points[:: args.stride]
    timestamps_ns = timestamps_ns[:: args.stride]

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.scatter(
        enu_points[:, 0],
        enu_points[:, 1],
        c="tab:blue",
        s=22,
        edgecolors="black",
        linewidths=0.3,
        label="sampled OXTS",
    )
    ax.scatter(enu_points[0, 0], enu_points[0, 1], c="red", s=40, label="start")
    if not args.hide_gap_labels:
        annotate_gaps(ax, enu_points, timestamps_ns)

    if enu_points.shape[0] >= 2:
        gaps = np.linalg.norm(np.diff(enu_points[:, :2], axis=0), axis=1)
        print(
            f"Sampled {enu_points.shape[0]} points with stride {args.stride}. "
            f"Mean 2D gap: {gaps.mean():.2f} m, max gap: {gaps.max():.2f} m"
        )
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title(args.title)
    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best")
    fig.tight_layout()

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=200)
        print(f"Saved plot to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()