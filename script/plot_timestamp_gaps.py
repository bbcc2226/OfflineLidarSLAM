#!/usr/bin/env python3

import argparse
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def parse_timestamp_ns(text: str) -> int:
    sec_str, nano_str = text.split('.')
    dt = datetime.strptime(sec_str, '%Y-%m-%d %H:%M:%S')
    return int(dt.timestamp() * 1e9) + int(nano_str.ljust(9, '0'))


def load_timestamps_ns(path: Path) -> np.ndarray:
    lines = [line.strip() for line in path.read_text().splitlines() if line.strip()]
    if len(lines) < 2:
        raise ValueError(f'Need at least two timestamps in {path}')
    return np.asarray([parse_timestamp_ns(line) for line in lines], dtype=np.int64)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Plot timestamp intervals and highlight abnormal gaps for KITTI timestamp files.'
    )
    parser.add_argument(
        'timestamps_path',
        nargs='?',
        default='/root/ros2_ws/data/2011_10_03_drive_0027/velodyne_points/timestamps.txt',
        help='Path to timestamps.txt.',
    )
    parser.add_argument(
        '--gap-threshold',
        type=float,
        default=0.05,
        help='Highlight intervals whose absolute deviation from the median exceeds this value in seconds.',
    )
    parser.add_argument(
        '--save',
        type=Path,
        help='Optional output image path. If omitted, show the plot interactively.',
    )
    parser.add_argument(
        '--title',
        default='Timestamp interval plot',
        help='Plot title.',
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    timestamps_path = Path(args.timestamps_path)
    timestamps_ns = load_timestamps_ns(timestamps_path)

    dt_sec = np.diff(timestamps_ns) / 1e9
    frame_idx = np.arange(dt_sec.shape[0])
    median_dt = float(np.median(dt_sec))
    abnormal_mask = np.abs(dt_sec - median_dt) > args.gap_threshold

    print(f'count={timestamps_ns.shape[0]}')
    print(f'min_dt={dt_sec.min():.6f}s')
    print(f'max_dt={dt_sec.max():.6f}s')
    print(f'mean_dt={dt_sec.mean():.6f}s')
    print(f'median_dt={median_dt:.6f}s')
    print(f'abnormal_count={int(abnormal_mask.sum())}')

    for idx in frame_idx[abnormal_mask][:20]:
        print(f'gap_idx={idx}->{idx + 1} dt={dt_sec[idx]:.6f}s')

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(frame_idx, dt_sec, linewidth=0.8, color='tab:blue', label='dt')
    ax.axhline(median_dt, color='tab:green', linestyle='--', linewidth=1.0, label=f'median {median_dt:.6f}s')

    if abnormal_mask.any():
        ax.scatter(frame_idx[abnormal_mask], dt_sec[abnormal_mask], color='tab:red', s=18, label='abnormal gap')
        for idx in frame_idx[abnormal_mask][:20]:
            ax.text(frame_idx[idx], dt_sec[idx], f'{dt_sec[idx]:.3f}s', fontsize=7, ha='left', va='bottom')

    ax.set_xlabel('Frame index')
    ax.set_ylabel('Delta time [s]')
    ax.set_title(args.title)
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(loc='best')
    fig.tight_layout()

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=200)
        print(f'Saved plot to {args.save}')
    else:
        plt.show()


if __name__ == '__main__':
    main()