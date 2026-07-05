#!/usr/bin/env python3
import ast
import struct
from collections import deque, Counter

MAP = "data_maps/benchmark_map_binary.npy"

RES_CM = 10
START_X_CM = 225
START_Y_CM = 225
START_Z_CM = 5

def load_npy(path):
    with open(path, "rb") as f:
        if f.read(6) != b"\x93NUMPY":
            raise ValueError("not npy")
        major, minor = f.read(2)
        if major == 1:
            header_len = struct.unpack("<H", f.read(2))[0]
        else:
            header_len = struct.unpack("<I", f.read(4))[0]
        header = f.read(header_len).decode("latin1").strip()
        meta = ast.literal_eval(header)
        shape = tuple(meta["shape"])
        descr = meta["descr"]
        raw = f.read()

    if descr not in ("|u1", "<u1", "u1"):
        raise ValueError(f"unsupported dtype {descr}")

    return shape, list(raw)

def idx(shape, z, y, x):
    depth, height, width = shape
    return (z * height + y) * width + x

def in_bounds(shape, z, y, x):
    depth, height, width = shape
    return 0 <= z < depth and 0 <= y < height and 0 <= x < width

def main():
    shape, data = load_npy(MAP)
    depth, height, width = shape

    sx = round(START_X_CM / RES_CM)
    sy = round(START_Y_CM / RES_CM)
    sz = round(START_Z_CM / RES_CM)

    print("shape:", shape)
    print("start cell approx:", f"z={sz}, y={sy}, x={sx}")
    print("start value:", data[idx(shape, sz, sy, sx)] if in_bounds(shape, sz, sy, sx) else "OOB")

    total_empty = sum(1 for v in data if v == 0)
    total_occupied = sum(1 for v in data if v == 1)

    print("total empty:", total_empty)
    print("total occupied:", total_occupied)

    # If the exact rounded start is not empty, find closest empty cell near it.
    start = None
    best_dist = 999999
    for z in range(depth):
        for y in range(height):
            for x in range(width):
                if data[idx(shape, z, y, x)] == 0:
                    d = abs(z - sz) + abs(y - sy) + abs(x - sx)
                    if d < best_dist:
                        best_dist = d
                        start = (z, y, x)

    print("nearest empty start:", start, "manhattan distance:", best_dist)

    q = deque([start])
    seen = {start}

    dirs = [
        (1, 0, 0), (-1, 0, 0),
        (0, 1, 0), (0, -1, 0),
        (0, 0, 1), (0, 0, -1),
    ]

    while q:
        z, y, x = q.popleft()
        for dz, dy, dx in dirs:
            nz, ny, nx = z + dz, y + dy, x + dx
            if not in_bounds(shape, nz, ny, nx):
                continue
            if (nz, ny, nx) in seen:
                continue
            if data[idx(shape, nz, ny, nx)] != 0:
                continue
            seen.add((nz, ny, nx))
            q.append((nz, ny, nx))

    reachable_empty = len(seen)
    print("reachable empty:", reachable_empty)
    print("reachable empty percent of all empty:", f"{100 * reachable_empty / max(1, total_empty):.2f}%")

    per_z = Counter(z for z, y, x in seen)
    print()
    print("reachable empty per z:")
    for z in range(depth):
        print(f"z={z:2d}: {per_z[z]:4d}")

if __name__ == "__main__":
    main()
