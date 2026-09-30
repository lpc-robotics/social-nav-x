#!/usr/bin/env python3
"""Generate the occupancy image from the same rectangular world definition as Isaac."""

from __future__ import annotations

import argparse
import struct
import zlib
from pathlib import Path

import yaml


def png_chunk(kind: bytes, payload: bytes) -> bytes:
    body = kind + payload
    return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default=str(root / "config/world/arena.yaml"))
    parser.add_argument("--output", default=str(root / "config/maps/map.png"))
    args = parser.parse_args()
    world = yaml.safe_load(Path(args.world).read_text(encoding="utf-8"))
    width_m = float(world["width"])
    height_m = float(world["height"])
    thickness = float(world["wall_thickness"])
    resolution = float(world["map_resolution"])
    margin = float(world["map_margin"])
    width = round((width_m + 2.0 * margin) / resolution) + 1
    height = round((height_m + 2.0 * margin) / resolution) + 1

    rows = []
    for row in range(height):
        y = -margin + (height - row - 0.5) * resolution
        pixels = bytearray()
        for column in range(width):
            x = -margin + (column + 0.5) * resolution
            inside = thickness / 2.0 < x < width_m - thickness / 2.0 \
                and thickness / 2.0 < y < height_m - thickness / 2.0
            pixels.append(255 if inside else 0)
        rows.append(b"\x00" + bytes(pixels))
    png = b"\x89PNG\r\n\x1a\n"
    png += png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
    png += png_chunk(b"IDAT", zlib.compress(b"".join(rows), level=9))
    png += png_chunk(b"IEND", b"")
    Path(args.output).write_bytes(png)
    print(f"generated {args.output}: {width}x{height}, resolution={resolution}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
