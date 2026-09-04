#!/usr/bin/env python3
import argparse
import math
import os
from pathlib import Path

import numpy as np
from PIL import Image


PCD_PATH = Path("/home/xixi/Unknow-gen0_humble/gen0_gz_sim_ros2/gen0_main/maps/prior_map.pcd")
OUT_DIR = PCD_PATH.parent
OUT_PREFIX = "prior_map_2d"
RESOLUTION = 0.1
OCCUPIED_THRESHOLD = 1
MIN_Z = 0.2
MAX_Z = 20.0


def read_ascii_pcd(path: Path):
    """Read x/y/z from an ASCII or standard binary PCD file."""
    raw = path.read_bytes()
    header_end = raw.find(b"DATA")
    if header_end < 0:
        raise ValueError(f"No DATA header found in {path}")

    data_line_end = raw.find(b"\n", header_end)
    if data_line_end < 0:
        raise ValueError(f"DATA header is incomplete in {path}")

    header = raw[:data_line_end].decode("ascii", errors="strict")
    header_lines = header.splitlines()
    fields = None
    sizes = None
    types = None
    counts = None
    points_count = None
    data_type = None
    for line in header_lines:
        parts = line.strip().split()
        if not parts:
            continue
        key = parts[0].upper()
        if key == "FIELDS":
            fields = parts[1:]
        elif key == "SIZE":
            sizes = [int(value) for value in parts[1:]]
        elif key == "TYPE":
            types = parts[1:]
        elif key == "COUNT":
            counts = [int(value) for value in parts[1:]]
        elif key == "POINTS":
            points_count = int(parts[1])
        elif key == "DATA":
            data_type = parts[1].lower()

    if not fields or not sizes or not types or not data_type:
        raise ValueError(f"Incomplete PCD header in {path}")
    if counts is None:
        counts = [1] * len(fields)
    if data_type == "ascii":
        text = raw[data_line_end + 1:].decode("ascii", errors="strict")
        values = np.fromstring(text, sep=" ", dtype=np.float64)
        columns = sum(counts)
        if columns <= 0 or values.size % columns != 0:
            raise ValueError(f"Invalid ASCII point data in {path}")
        values = values.reshape((-1, columns))
        field_offsets = np.cumsum([0] + counts)
        xyz_indices = [fields.index(name) for name in ("x", "y", "z")]
        return values[:, [field_offsets[index] for index in xyz_indices]]
    if data_type != "binary":
        raise ValueError(f"Unsupported PCD DATA type {data_type!r} in {path}")

    if points_count is None:
        width_index = next((i for i, line in enumerate(header_lines) if line.startswith("WIDTH")), None)
        height_index = next((i for i, line in enumerate(header_lines) if line.startswith("HEIGHT")), None)
        if width_index is None or height_index is None:
            raise ValueError(f"Binary PCD has no POINTS/WIDTH/HEIGHT in {path}")
        points_count = int(header_lines[width_index].split()[1]) * int(header_lines[height_index].split()[1])

    dtype_fields = []
    for field, size, field_type, count in zip(fields, sizes, types, counts):
        if field_type == "F" and size == 4:
            numpy_type = "<f4"
        elif field_type == "F" and size == 8:
            numpy_type = "<f8"
        elif field_type == "U":
            numpy_type = f"<u{size}"
        elif field_type == "I":
            numpy_type = f"<i{size}"
        else:
            raise ValueError(f"Unsupported PCD field type {field_type}{size} in {path}")
        shape = () if count == 1 else (count,)
        dtype_fields.append((field, numpy_type, shape))

    structured = np.frombuffer(
        raw[data_line_end + 1:], dtype=np.dtype(dtype_fields), count=points_count
    )
    try:
        return np.column_stack((structured["x"], structured["y"], structured["z"])).astype(
            np.float64, copy=False
        )
    except (KeyError, ValueError) as error:
        raise ValueError(f"Binary PCD does not contain scalar x/y/z fields: {path}") from error


def project_to_2d(points):
    pts = points.copy()
    if pts.shape[1] >= 4:
        # keep intensity in the 4th column but filter by z only
        pts = pts[:, :4]
    else:
        pts = np.column_stack([pts, np.ones(len(pts), dtype=np.float64)])

    z_mask = (pts[:, 2] >= MIN_Z) & (pts[:, 2] <= MAX_Z)
    pts = pts[z_mask]
    if pts.shape[0] == 0:
        raise RuntimeError(f"No points remain after filtering z in [{MIN_Z}, {MAX_Z}]")

    min_x = float(np.min(pts[:, 0]))
    min_y = float(np.min(pts[:, 1]))
    max_x = float(np.max(pts[:, 0]))
    max_y = float(np.max(pts[:, 1]))

    width = int(math.ceil((max_x - min_x) / RESOLUTION)) + 1
    height = int(math.ceil((max_y - min_y) / RESOLUTION)) + 1
    # Nav2 with negate: 0 interprets black as occupied and white as free.
    grid = np.zeros((height, width), dtype=np.uint16)

    for x, y, *_ in pts:
        gx = int((x - min_x) / RESOLUTION)
        gy = int((y - min_y) / RESOLUTION)
        if 0 <= gx < width and 0 <= gy < height:
            grid[height - 1 - gy, gx] += 1

    # Keep the Nav2 occupancy meaning unchanged while rendering occupied cells white.
    occupied = np.where(grid >= OCCUPIED_THRESHOLD, 255, 0).astype(np.uint8)
    return occupied, min_x, min_y


def save_yaml(yaml_path: Path, image_name: str, origin_x: float, origin_y: float):
    yaml_content = (
        f"image: {image_name}\n"
        "mode: trinary\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\n"
        "negate: 1\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    temporary_path = yaml_path.with_name(f".{yaml_path.name}.tmp")
    temporary_path.write_text(yaml_content, encoding="utf-8")
    os.replace(temporary_path, yaml_path)


def parse_args():
    parser = argparse.ArgumentParser(description="Convert a PCD point cloud to a Nav2 map.")
    parser.add_argument("--pcd", type=Path, default=PCD_PATH)
    parser.add_argument("--output-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--prefix", default=OUT_PREFIX)
    parser.add_argument("--image-format", choices=("pgm", "png"), default="pgm")
    return parser.parse_args()


def main():
    args = parse_args()
    pcd_path = args.pcd.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if not pcd_path.is_file():
        raise FileNotFoundError(f"PCD not found: {pcd_path}")

    points = read_ascii_pcd(pcd_path)
    img, origin_x, origin_y = project_to_2d(points)

    image_path = output_dir / f"{args.prefix}.{args.image_format}"
    yaml_path = output_dir / f"{args.prefix}.yaml"

    temporary_image_path = image_path.with_name(
        f".{image_path.name}.tmp.{args.image_format}"
    )
    Image.fromarray(img, mode="L").save(temporary_image_path, format=args.image_format.upper())
    os.replace(temporary_image_path, image_path)
    save_yaml(yaml_path, image_path.name, origin_x, origin_y)

    print(f"Saved 2D occupancy map to: {image_path}")
    print(f"Saved YAML map to: {yaml_path}")
    print(f"Origin: ({origin_x}, {origin_y}), resolution={RESOLUTION} m/cell")


if __name__ == "__main__":
    main()
