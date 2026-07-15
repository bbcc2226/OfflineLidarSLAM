#!/usr/bin/env python3
"""Associate keyframe records with nearest KITTI camera images by timestamp.

This script reads keyframe records from a JSONL file (supports multiline JSON objects),
reads KITTI image timestamps, and writes an enriched JSONL output where each keyframe has
its nearest image match.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


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
    object_texts = extract_json_objects(text)
    if not object_texts:
        return []

    records: List[Dict[str, Any]] = []
    for obj_text in object_texts:
        records.append(json.loads(obj_text))
    return records


def read_image_timestamps(image_timestamps_path: Path) -> List[float]:
    lines = image_timestamps_path.read_text(encoding="utf-8").splitlines()
    timestamps: List[float] = []
    for line in lines:
        if line.strip():
            timestamps.append(parse_kitti_time_to_sec(line))
    return timestamps


@dataclass
class AssocResult:
    image_index: int
    image_timestamp_sec: float
    delta_sec: float


def nearest_index(sorted_values: Sequence[float], target: float) -> int:
    if not sorted_values:
        raise ValueError("Cannot search in empty sequence")

    lo = 0
    hi = len(sorted_values) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if sorted_values[mid] < target:
            lo = mid + 1
        else:
            hi = mid

    idx = lo
    if idx == 0:
        return 0
    if idx == len(sorted_values):
        return len(sorted_values) - 1

    prev_idx = idx - 1
    if abs(sorted_values[prev_idx] - target) <= abs(sorted_values[idx] - target):
        return prev_idx
    return idx


def associate_keyframe(
    keyframe_timestamp_sec: float,
    image_timestamps_sec: Sequence[float],
) -> AssocResult:
    idx = nearest_index(image_timestamps_sec, keyframe_timestamp_sec)
    image_ts = image_timestamps_sec[idx]
    return AssocResult(
        image_index=idx,
        image_timestamp_sec=image_ts,
        delta_sec=abs(image_ts - keyframe_timestamp_sec),
    )


def write_jsonl(records: Sequence[Dict[str, Any]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        for rec in records:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Associate keyframe JSONL records with nearest KITTI image timestamps"
    )
    parser.add_argument(
        "--keyframes",
        type=Path,
        required=True,
        help="Path to key_frames.jsonl",
    )
    parser.add_argument(
        "--image-dir",
        type=Path,
        required=True,
        help="KITTI image_02 directory (must contain timestamps.txt and data/)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("key_frames_with_images.jsonl"),
        help="Output enriched JSONL path",
    )
    parser.add_argument(
        "--max-gap-ms",
        type=float,
        default=120.0,
        help="Maximum allowed association gap in milliseconds",
    )
    args = parser.parse_args()

    keyframes_path: Path = args.keyframes
    image_dir: Path = args.image_dir
    output_path: Path = args.output
    max_gap_sec = args.max_gap_ms * 1e-3

    timestamps_path = image_dir / "timestamps.txt"
    data_dir = image_dir / "data"

    if not keyframes_path.exists():
        raise FileNotFoundError(f"Keyframe JSONL not found: {keyframes_path}")
    if not timestamps_path.exists():
        raise FileNotFoundError(f"Image timestamps not found: {timestamps_path}")
    if not data_dir.exists():
        raise FileNotFoundError(f"Image data directory not found: {data_dir}")

    keyframes = read_keyframes_jsonl(keyframes_path)
    image_timestamps = read_image_timestamps(timestamps_path)

    if not keyframes:
        print("No keyframe records found.")
        write_jsonl([], output_path)
        return
    if not image_timestamps:
        raise RuntimeError("No image timestamps found.")

    enriched: List[Dict[str, Any]] = []
    matched_count = 0

    for rec in keyframes:
        rec_out = dict(rec)

        keyframe_ts = float(rec.get("timestamp", 0.0))
        assoc = associate_keyframe(keyframe_ts, image_timestamps)

        image_name = f"{assoc.image_index:010d}.png"
        image_path = data_dir / image_name
        has_image = image_path.exists()
        accepted = assoc.delta_sec <= max_gap_sec and has_image

        rec_out["image_assoc"] = {
            "image_index": assoc.image_index,
            "image_timestamp": float(f"{assoc.image_timestamp_sec:.15f}"),
            "image_path": str(image_path),
            "delta_sec": float(f"{assoc.delta_sec:.15f}"),
            "accepted": accepted,
        }

        if accepted:
            matched_count += 1
        enriched.append(rec_out)

    write_jsonl(enriched, output_path)

    print(
        "Associated keyframes with images: "
        f"total={len(enriched)}, accepted={matched_count}, "
        f"rejected={len(enriched) - matched_count}, output={output_path}"
    )


if __name__ == "__main__":
    main()
