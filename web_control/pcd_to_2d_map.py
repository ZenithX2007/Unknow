#!/usr/bin/env python3
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
MIN_Z = -0.2
MAX_Z = 20.0


def read_ascii_pcd(path: Path):
    """Read a simple ASCII .pcd file with x y z [intensity]."""
    with path.open("r", encoding="utf-8") as fp:
        lines = fp.readlines()

    fields = None
    data_started = False
    data_lines = []

    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if not data_started:
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("DATA"):
                data_type = line.split()[1]
                if data_type.lower() != "ascii":
                    raise ValueError(f"Only ASCII PCD is supported: {path}")
                data_started = True
            continue

        if data_started:
            data_lines.append(line)

    if fields is None:
        raise ValueError(f"No FIELDS header found in {path}")

    if not data_lines:
        raise ValueError(f"No point data found in {path}")

    points = []
    for line in data_lines:
        vals = line.split()
        if len(vals) < 3:
            continue
        # Keep exactly x, y, z and optionally intensity
        if len(vals) >= 4:
            points.append([float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3])])
        else:
            points.append([float(vals[0]), float(vals[1]), float(vals[2])])

    if not points:
        raise ValueError(f"No valid points parsed from {path}")

    arr = np.asarray(points, dtype=np.float64)
    if arr.shape[1] == 3:
        return arr
    if arr.shape[1] == 4:
        return arr
    raise ValueError(f"Unexpected point format in {path}: shape={arr.shape}")


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

    occupied = np.where(grid >= OCCUPIED_THRESHOLD, 0, 255).astype(np.uint8)
    return occupied, min_x, min_y


def save_yaml(yaml_path: Path, image_name: str, origin_x: float, origin_y: float):
    yaml_path.write_text(
        f"image: {image_name}\n"
        "mode: trinary\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8",
    )


def main():
    if not PCD_PATH.exists():
        raise FileNotFoundError(f"PCD not found: {PCD_PATH}")

    points = read_ascii_pcd(PCD_PATH)
    img, origin_x, origin_y = project_to_2d(points)

    pgm_path = OUT_DIR / f"{OUT_PREFIX}.pgm"
    yaml_path = OUT_DIR / f"{OUT_PREFIX}.yaml"

    Image.fromarray(img, mode="L").save(pgm_path)
    save_yaml(yaml_path, f"{OUT_PREFIX}.pgm", origin_x, origin_y)

    print(f"Saved 2D occupancy map to: {pgm_path}")
    print(f"Saved YAML map to: {yaml_path}")
    print(f"Origin: ({origin_x}, {origin_y}), resolution={RESOLUTION} m/cell")


if __name__ == "__main__":
    main()
