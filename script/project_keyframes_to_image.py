#!/usr/bin/env python3
"""Project keyframe LiDAR clouds into KITTI image_02 frames.

This utility helps verify LiDAR-to-camera projection quality by generating overlay
images and optional per-point RGB association files.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw


DEFAULT_EXTRINSIC_OFFSET_X = 0.0
DEFAULT_EXTRINSIC_OFFSET_Y = 0.0
DEFAULT_EXTRINSIC_OFFSET_Z = -1.6


def parse_kitti_time_to_sec(line: str) -> float:
    text = line.strip()
    if not text:
        raise ValueError("Empty KITTI timestamp line")
    if "." not in text:
        dt = datetime.strptime(text, "%Y-%m-%d %H:%M:%S")
        return dt.replace(tzinfo=timezone.utc).timestamp()
    sec_str, frac_str = text.split(".", 1)
    dt = datetime.strptime(sec_str, "%Y-%m-%d %H:%M:%S")
    base = dt.replace(tzinfo=timezone.utc).timestamp()
    nanos = int(frac_str.ljust(9, "0")[:9])
    return base + nanos * 1e-9


def extract_json_objects(text: str) -> List[str]:
    objects: List[str] = []
    depth = 0
    in_string = False
    escaped = False
    start_idx: Optional[int] = None

    for idx, ch in enumerate(text):
        if in_string:
            if escaped:
                escaped = False
            elif ch == "\\":
                escaped = True
            elif ch == '"':
                in_string = False
            continue

        if ch == '"':
            in_string = True
            continue

        if ch == "{":
            if depth == 0:
                start_idx = idx
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0 and start_idx is not None:
                objects.append(text[start_idx : idx + 1])
                start_idx = None

    return objects


def read_keyframes_jsonl(path: Path) -> List[Dict[str, Any]]:
    text = path.read_text(encoding="utf-8")
    return [json.loads(obj) for obj in extract_json_objects(text)]


def read_image_timestamps(path: Path) -> List[float]:
    values: List[float] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            values.append(parse_kitti_time_to_sec(line))
    return values


def nearest_index(sorted_values: Sequence[float], target: float) -> int:
    idx = bisect.bisect_left(sorted_values, target)
    if idx <= 0:
        return 0
    if idx >= len(sorted_values):
        return len(sorted_values) - 1
    prev_idx = idx - 1
    if abs(sorted_values[prev_idx] - target) <= abs(sorted_values[idx] - target):
        return prev_idx
    return idx


def parse_kitti_calib_file(path: Path) -> Dict[str, np.ndarray]:
    out: Dict[str, np.ndarray] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        parts = value.strip().split()
        if not parts:
            continue
        try:
            arr = np.array([float(v) for v in parts], dtype=np.float64)
            out[key] = arr
        except ValueError:
            continue
    return out


def load_projection_mats(
    dataset_root: Path,
    rect_key: str,
    extrinsic_offset_cam: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    cam_calib = parse_kitti_calib_file(dataset_root / "calib_cam_to_cam.txt")
    velo_calib = parse_kitti_calib_file(dataset_root / "calib_velo_to_cam.txt")

    if "P_rect_02" not in cam_calib:
        raise KeyError("Missing P_rect_02 in calib_cam_to_cam.txt")
    p2 = cam_calib["P_rect_02"].reshape(3, 4)

    r_rect = cam_calib.get(rect_key, np.eye(3).reshape(-1))
    if r_rect.size != 9:
        raise ValueError(f"{rect_key} must contain 9 values")
    r_rect_4 = np.eye(4, dtype=np.float64)
    r_rect_4[:3, :3] = r_rect.reshape(3, 3)

    r = velo_calib.get("R")
    t = velo_calib.get("T")
    if r is None or t is None or r.size != 9 or t.size != 3:
        raise ValueError("calib_velo_to_cam.txt must contain R(9) and T(3)")

    tr_velo_to_cam = np.eye(4, dtype=np.float64)
    tr_velo_to_cam[:3, :3] = r.reshape(3, 3)
    tr_velo_to_cam[:3, 3] = t.reshape(3)
    # tr_velo_to_cam[:3, 3] += extrinsic_offset_cam

    t_cam_from_velo = r_rect_4 @ tr_velo_to_cam
    return p2, t_cam_from_velo


def read_ascii_ply_vertices(ply_path: Path) -> np.ndarray:
    with ply_path.open("r", encoding="utf-8") as handle:
        line = handle.readline().strip()
        if line != "ply":
            raise ValueError(f"Not a PLY file: {ply_path}")

        vertex_count = None
        format_name = None
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"Invalid PLY header: {ply_path}")
            line = line.strip()
            if line.startswith("format"):
                parts = line.split()
                if len(parts) >= 2:
                    format_name = parts[1]
            elif line.startswith("element vertex"):
                vertex_count = int(line.split()[-1])
            elif line == "end_header":
                break

        if format_name != "ascii":
            raise ValueError(f"Only ASCII PLY is supported: {ply_path}")
        if vertex_count is None or vertex_count <= 0:
            return np.empty((0, 3), dtype=np.float64)

        pts: List[Tuple[float, float, float]] = []
        for _ in range(vertex_count):
            raw = handle.readline()
            if not raw:
                break
            parts = raw.strip().split()
            if len(parts) < 3:
                continue
            pts.append((float(parts[0]), float(parts[1]), float(parts[2])))

    if not pts:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(pts, dtype=np.float64)


def write_ascii_ply_xyzrgb(ply_path: Path, points_xyz: np.ndarray, rgb: np.ndarray) -> None:
    if points_xyz.shape[0] != rgb.shape[0]:
        raise ValueError("points_xyz and rgb must have the same number of rows")

    ply_path.parent.mkdir(parents=True, exist_ok=True)
    with ply_path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {points_xyz.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for i in range(points_xyz.shape[0]):
            x, y, z = points_xyz[i]
            r, g, b = rgb[i]
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")


def resolve_cloud_path(keyframes_path: Path, saved_frame_path: str) -> Path:
    cloud_path = Path(saved_frame_path)
    if cloud_path.is_absolute():
        return cloud_path
    candidates = [
        (keyframes_path.parent / cloud_path).resolve(),
        (keyframes_path.parent.parent / cloud_path).resolve(),
        cloud_path.resolve(),
    ]
    for p in candidates:
        if p.is_file():
            return p
    return candidates[0]


def depth_to_rgb(depth: np.ndarray, depth_min: float, depth_max: float) -> np.ndarray:
    if depth_max <= depth_min:
        depth_max = depth_min + 1e-6
    t = np.clip((depth - depth_min) / (depth_max - depth_min), 0.0, 1.0)
    r = (255.0 * (1.0 - t)).astype(np.uint8)
    g = (255.0 * (1.0 - np.abs(2.0 * t - 1.0))).astype(np.uint8)
    b = (255.0 * t).astype(np.uint8)
    return np.stack([r, g, b], axis=1)


def project_points(
    points_velo: np.ndarray,
    p2: np.ndarray,
    t_cam_from_velo: np.ndarray,
    image_w: int,
    image_h: int,
    min_depth: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if points_velo.size == 0:
        return (
            np.empty((0, 2), dtype=np.int32),
            np.empty((0,), dtype=np.float64),
            np.empty((0,), dtype=np.int64),
        )

    n = points_velo.shape[0]
    pts_h = np.concatenate([points_velo, np.ones((n, 1), dtype=np.float64)], axis=1)
    pts_cam_h = (t_cam_from_velo @ pts_h.T).T
    z = pts_cam_h[:, 2]
    front_mask = z > min_depth
    valid_idx = np.where(front_mask)[0]
    if valid_idx.size == 0:
        return (
            np.empty((0, 2), dtype=np.int32),
            np.empty((0,), dtype=np.float64),
            np.empty((0,), dtype=np.int64),
        )

    proj = (p2 @ pts_cam_h[front_mask].T).T
    w = proj[:, 2]
    nonzero_mask = np.abs(w) > 1e-9
    valid_idx = valid_idx[nonzero_mask]
    proj = proj[nonzero_mask]
    if valid_idx.size == 0:
        return (
            np.empty((0, 2), dtype=np.int32),
            np.empty((0,), dtype=np.float64),
            np.empty((0,), dtype=np.int64),
        )

    u = proj[:, 0] / proj[:, 2]
    v = proj[:, 1] / proj[:, 2]

    in_img = (u >= 0.0) & (u < image_w) & (v >= 0.0) & (v < image_h)
    valid_idx = valid_idx[in_img]
    u = u[in_img]
    v = v[in_img]

    uv = np.stack([np.round(u).astype(np.int32), np.round(v).astype(np.int32)], axis=1)
    rounded_in_img = (
        (uv[:, 0] >= 0)
        & (uv[:, 0] < image_w)
        & (uv[:, 1] >= 0)
        & (uv[:, 1] < image_h)
    )
    uv = uv[rounded_in_img]
    valid_idx = valid_idx[rounded_in_img]
    depth = z[valid_idx]
    return uv, depth, valid_idx


def zbuffer_keep_nearest_per_pixel(
    uv: np.ndarray,
    depth: np.ndarray,
    valid_idx: np.ndarray,
    image_w: int,
    image_h: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if uv.shape[0] == 0:
        return uv, depth, valid_idx

    flat = uv[:, 1].astype(np.int64) * int(image_w) + uv[:, 0].astype(np.int64)
    order = np.argsort(depth)
    flat_sorted = flat[order]

    # Keep first occurrence in depth-sorted order -> nearest point for each pixel.
    keep_first = np.ones(flat_sorted.shape[0], dtype=bool)
    keep_first[1:] = flat_sorted[1:] != flat_sorted[:-1]
    keep_indices = order[keep_first]

    # Stable visualization order: far to near so closer points draw on top.
    draw_order = np.argsort(depth[keep_indices])[::-1]
    keep_indices = keep_indices[draw_order]

    return uv[keep_indices], depth[keep_indices], valid_idx[keep_indices]


def find_image_for_record(
    record: Dict[str, Any],
    image_dir: Path,
    image_timestamps: Sequence[float],
) -> Tuple[Path, float, float, int]:
    assoc = record.get("image_assoc")
    if isinstance(assoc, dict):
        img_path = assoc.get("image_path")
        if isinstance(img_path, str) and Path(img_path).is_file():
            return Path(img_path), float(assoc.get("image_timestamp", 0.0)), float(assoc.get("delta_sec", 0.0)), int(assoc.get("image_index", -1))

    keyframe_ts = float(record.get("timestamp", 0.0))
    idx = nearest_index(image_timestamps, keyframe_ts)
    image_ts = image_timestamps[idx]
    image_path = image_dir / "data" / f"{idx:010d}.png"
    return image_path, image_ts, abs(image_ts - keyframe_ts), idx


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Project keyframe LiDAR clouds into KITTI image_02 and save overlays.")
    parser.add_argument("--keyframes", type=Path, default=Path("/root/ros2_ws/build/offline_lidar_slam/key_frames.jsonl"), help="Path to keyframe jsonl")
    parser.add_argument("--dataset-root", type=Path, default=Path("/root/ros2_ws/data/2011_10_03_drive_0027"), help="KITTI drive root with calib_cam_to_cam.txt and calib_velo_to_cam.txt")
    parser.add_argument("--image-dir", type=Path, default=None, help="image_02 directory; defaults to <dataset-root>/image_02")
    parser.add_argument("--output-dir", type=Path, default=Path("/root/ros2_ws/build/offline_lidar_slam/projection_check"), help="Output directory")
    parser.add_argument("--max-gap-ms", type=float, default=10.0, help="Max keyframe-image association gap in ms")
    parser.add_argument("--max-frames", type=int, default=200, help="Max keyframes to process")
    parser.add_argument("--draw-max-points", type=int, default=25000, help="Max projected points to draw per image")
    parser.add_argument("--point-radius", type=int, default=1, help="Overlay point radius in pixels")
    parser.add_argument("--min-depth", type=float, default=0.5, help="Minimum positive camera depth in meters")
    parser.add_argument("--save-rgb-association", action="store_true", help="Save per-keyframe projected uv/rgb/depth npz")
    parser.add_argument(
        "--save-colored-cloud",
        action="store_true",
        help="Save per-keyframe projected LiDAR points with associated image RGB as ASCII PLY",
    )
    parser.add_argument("--save-all", action="store_true", help="Save overlays even when timestamp gap exceeds max-gap-ms")
    parser.add_argument(
        "--disable-z-buffer-overlay",
        action="store_true",
        help="Disable nearest-depth-per-pixel filtering for overlay rendering",
    )
    parser.add_argument(
        "--rect-key",
        type=str,
        default="R_rect_00",
        choices=["R_rect_00", "R_rect_02"],
        help="Rectification matrix key from calib_cam_to_cam.txt",
    )
    parser.add_argument("--extrinsic-offset-x", type=float, default=DEFAULT_EXTRINSIC_OFFSET_X, help="Extra camera-frame X offset added to Tr_velo_to_cam translation")
    parser.add_argument("--extrinsic-offset-y", type=float, default=DEFAULT_EXTRINSIC_OFFSET_Y, help="Extra camera-frame Y offset added to Tr_velo_to_cam translation")
    parser.add_argument("--extrinsic-offset-z", type=float, default=DEFAULT_EXTRINSIC_OFFSET_Z, help="Extra camera-frame Z offset added to Tr_velo_to_cam translation")
    parser.add_argument("--print-calib", action="store_true", help="Print projection matrices and adjusted extrinsic before processing")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    keyframes_path: Path = args.keyframes
    dataset_root: Path = args.dataset_root
    image_dir: Path = args.image_dir if args.image_dir is not None else dataset_root / "image_02"
    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    if not keyframes_path.is_file():
        raise FileNotFoundError(f"Keyframe jsonl not found: {keyframes_path}")
    if not image_dir.is_dir():
        raise FileNotFoundError(f"Image directory not found: {image_dir}")

    image_ts_file = image_dir / "timestamps.txt"
    if not image_ts_file.is_file():
        raise FileNotFoundError(f"Image timestamps file not found: {image_ts_file}")

    extrinsic_offset_cam = np.array(
        [args.extrinsic_offset_x, args.extrinsic_offset_y, args.extrinsic_offset_z],
        dtype=np.float64,
    )
    p2, t_cam_from_velo = load_projection_mats(dataset_root, args.rect_key, extrinsic_offset_cam)

    if args.print_calib:
        np.set_printoptions(precision=8, suppress=True)
        print(f"[CALIB] rect_key={args.rect_key}")
        print(f"[CALIB] P_rect_02=\n{p2}")
        print(f"[CALIB] T_cam_from_velo (after offset)=\n{t_cam_from_velo}")
        print(f"[CALIB] added_offset_cam={extrinsic_offset_cam.tolist()}")
    image_timestamps = read_image_timestamps(image_ts_file)
    records = read_keyframes_jsonl(keyframes_path)

    max_gap_sec = args.max_gap_ms * 1e-3
    processed = 0
    kept = 0
    summary: List[Dict[str, Any]] = []

    for rec in records:
        if processed >= args.max_frames:
            break

        key_frame_id = int(rec.get("key_frame_id", -1))
        saved_frame_path = str(rec.get("saved_frame_path", ""))
        if not saved_frame_path:
            continue

        image_path, image_ts, delta_sec, image_index = find_image_for_record(rec, image_dir, image_timestamps)
        if not image_path.is_file():
            continue

        accepted = delta_sec <= max_gap_sec
        if not accepted and not args.save_all:
            summary.append({
                "key_frame_id": key_frame_id,
                "status": "skipped_large_time_gap",
                "delta_sec": float(f"{delta_sec:.15f}"),
                "image_index": image_index,
                "image_path": str(image_path),
            })
            continue

        cloud_path = resolve_cloud_path(keyframes_path, saved_frame_path)
        if not cloud_path.is_file():
            summary.append({
                "key_frame_id": key_frame_id,
                "status": "missing_cloud",
                "cloud_path": str(cloud_path),
            })
            continue

        points = read_ascii_ply_vertices(cloud_path)
        if points.size == 0:
            summary.append({"key_frame_id": key_frame_id, "status": "empty_cloud"})
            continue

        image = np.array(Image.open(image_path).convert("RGB"), dtype=np.uint8)
        h, w = image.shape[:2]

        uv, depth, valid_idx = project_points(points, p2, t_cam_from_velo, w, h, args.min_depth)
        if uv.shape[0] == 0:
            summary.append({
                "key_frame_id": key_frame_id,
                "status": "no_projected_points",
                "cloud_points": int(points.shape[0]),
            })
            continue

        if not args.disable_z_buffer_overlay:
            uv, depth, valid_idx = zbuffer_keep_nearest_per_pixel(uv, depth, valid_idx, w, h)

        if uv.shape[0] > args.draw_max_points:
            step = int(math.ceil(uv.shape[0] / float(args.draw_max_points)))
            draw_idx = np.arange(0, uv.shape[0], step)
        else:
            draw_idx = np.arange(uv.shape[0])

        uv_draw = uv[draw_idx]
        depth_draw = depth[draw_idx]

        colors = depth_to_rgb(depth_draw, float(np.min(depth_draw)), float(np.max(depth_draw)))

        overlay_img = Image.fromarray(image.copy())
        drawer = ImageDraw.Draw(overlay_img)
        r = max(0, int(args.point_radius))
        for i in range(uv_draw.shape[0]):
            u = int(uv_draw[i, 0])
            v = int(uv_draw[i, 1])
            c = tuple(int(x) for x in colors[i])
            if r <= 0:
                drawer.point((u, v), fill=c)
            else:
                drawer.ellipse((u - r, v - r, u + r, v + r), fill=c)

        image_prefix = f"kf_{key_frame_id:06d}_img_{image_index:010d}"
        overlay_path = output_dir / f"{image_prefix}_overlay.png"
        overlay_img.save(overlay_path)

        uv_all = uv
        rgb_all = image[uv_all[:, 1], uv_all[:, 0], :]
        depth_all = depth
        points_lidar_assoc = points[valid_idx]

        if args.save_rgb_association:
            npz_path = output_dir / f"{image_prefix}_assoc.npz"
            np.savez_compressed(
                npz_path,
                key_frame_id=np.int32(key_frame_id),
                image_index=np.int32(image_index),
                points_lidar=points_lidar_assoc.astype(np.float32),
                uv=uv_all.astype(np.int32),
                rgb=rgb_all.astype(np.uint8),
                depth=depth_all.astype(np.float32),
            )

        colored_cloud_path = None
        if args.save_colored_cloud:
            colored_cloud_path = output_dir / f"{image_prefix}_colored.ply"
            write_ascii_ply_xyzrgb(colored_cloud_path, points_lidar_assoc, rgb_all)
            print(f"[PROJ] wrote colored cloud {colored_cloud_path}")

        summary.append(
            {
                "key_frame_id": key_frame_id,
                "status": "ok",
                "timestamp": float(f"{float(rec.get('timestamp', 0.0)):.15f}"),
                "image_timestamp": float(f"{image_ts:.15f}"),
                "delta_sec": float(f"{delta_sec:.15f}"),
                "accepted": bool(accepted),
                "cloud_path": str(cloud_path),
                "image_path": str(image_path),
                "overlay_path": str(overlay_path),
                "colored_cloud_path": str(colored_cloud_path) if colored_cloud_path is not None else None,
                "cloud_points": int(points.shape[0]),
                "projected_points": int(uv_all.shape[0]),
                "drawn_points": int(uv_draw.shape[0]),
            }
        )

        processed += 1
        kept += 1
        print(
            f"[PROJ] key_frame_id={key_frame_id} image={image_index} "
            f"delta={delta_sec:.6f}s projected={uv_all.shape[0]} drawn={uv_draw.shape[0]}"
        )

    summary_path = output_dir / "projection_summary.jsonl"
    with summary_path.open("w", encoding="utf-8") as f:
        for rec in summary:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

    print(
        "Projection check complete: "
        f"records={len(summary)} processed={processed} successful={kept} output={output_dir}"
    )


if __name__ == "__main__":
    main()
