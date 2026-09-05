"""Generate small, repeatable bitmap materials for the Gen0 CAD model."""

from __future__ import annotations

import math
from pathlib import Path
from random import Random
import struct
import zlib


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "gen0_gz_sim_ros2/gen0_main/meshes/gen0_cad"
SIZE = 512


def write_png(path: Path, pixels: bytes) -> None:
    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(
            ">I", zlib.crc32(kind + data) & 0xFFFFFFFF
        )

    rows = b"".join(b"\x00" + pixels[row * SIZE * 3 : (row + 1) * SIZE * 3] for row in range(SIZE))
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", SIZE, SIZE, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(rows, 9))
    png += chunk(b"IEND", b"")
    path.write_bytes(png)


def body_texture() -> bytes:
    random = Random(20260903)
    pixels = bytearray()
    for y in range(SIZE):
        for x in range(SIZE):
            # A low-contrast powder-coat grain. The sinusoid wraps at each edge.
            grain = 3.2 * math.sin(2 * math.pi * x / 19) * math.sin(2 * math.pi * y / 23)
            grain += random.uniform(-2.0, 2.0)
            base = max(0, min(255, round(25 + grain)))
            pixels.extend((max(0, base - 3), base, min(255, base + 3)))
    return bytes(pixels)


def hub_texture() -> bytes:
    random = Random(20260904)
    pixels = bytearray()
    for y in range(SIZE):
        for x in range(SIZE):
            dx = x - (SIZE - 1) / 2
            dy = y - (SIZE - 1) / 2
            radius = math.hypot(dx, dy)
            # A light, fine radial machining pattern suited to the hub UVs.
            machining = 7.0 * math.sin(radius * 1.15) + 2.5 * math.sin(radius * 4.4)
            machining += random.uniform(-1.5, 1.5)
            base = max(0, min(255, round(201 + machining)))
            pixels.extend((base - 5, base, min(255, base + 9)))
    return bytes(pixels)


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    write_png(OUTPUT_DIR / "body_black_softtouch.png", body_texture())
    write_png(OUTPUT_DIR / "hub_light_machined.png", hub_texture())


if __name__ == "__main__":
    main()
