#!/usr/bin/env python3
import ast
import struct
import math

MAP = "data_maps/benchmark_map_binary.npy"
RES_CM = 10
RADIUS_CM = 5

POINTS = {
    "current": (295, 5, 25),
    "target":  (285, 5, 25),
}

def load_npy(path):
    with open(path, "rb") as f:
        assert f.read(6) == b"\x93NUMPY"
        major, minor = f.read(2)
        header_len = struct.unpack("<H", f.read(2))[0] if major == 1 else struct.unpack("<I", f.read(4))[0]
        header = f.read(header_len).decode("latin1").strip()
        meta = ast.literal_eval(header)
        shape = tuple(meta["shape"])
        raw = list(f.read())
    return shape, raw

def idx(shape, z, y, x):
    depth, height, width = shape
    return (z * height + y) * width + x

def cell_from_cm(shape, x_cm, y_cm, z_cm):
    x = math.floor(x_cm / RES_CM)
    y = math.floor(y_cm / RES_CM)
    z = math.floor(z_cm / RES_CM)
    depth, height, width = shape
    if not (0 <= z < depth and 0 <= y < height and 0 <= x < width):
        return None
    return z, y, x

def value_at(shape, data, x_cm, y_cm, z_cm):
    cell = cell_from_cm(shape, x_cm, y_cm, z_cm)
    if cell is None:
        return 1
    z, y, x = cell
    return data[idx(shape, z, y, x)]

def legal_center(shape, data, x_cm, y_cm, z_cm):
    for dz in range(-RADIUS_CM, RADIUS_CM + 1):
        for dy in range(-RADIUS_CM, RADIUS_CM + 1):
            for dx in range(-RADIUS_CM, RADIUS_CM + 1):
                if dx*dx + dy*dy + dz*dz > RADIUS_CM * RADIUS_CM:
                    continue
                if value_at(shape, data, x_cm + dx, y_cm + dy, z_cm + dz) != 0:
                    return False
    return True

def legal_path(shape, data, start, end):
    x1, y1, z1 = start
    x2, y2, z2 = end
    steps = int(max(abs(x2-x1), abs(y2-y1), abs(z2-z1)))
    for i in range(steps + 1):
        t = i / max(1, steps)
        x = x1 + (x2 - x1) * t
        y = y1 + (y2 - y1) * t
        z = z1 + (z2 - z1) * t
        if not legal_center(shape, data, x, y, z):
            print("first illegal path sample:", round(x,2), round(y,2), round(z,2))
            return False
    return True

shape, data = load_npy(MAP)
print("shape:", shape)

for name, p in POINTS.items():
    x, y, z = p
    print()
    print(name, "center cm:", p)
    print(name, "cell:", cell_from_cm(shape, x, y, z))
    print(name, "center voxel value:", value_at(shape, data, x, y, z))
    print(name, "legal for drone body:", legal_center(shape, data, x, y, z))

print()
print("path current -> target legal:", legal_path(shape, data, POINTS["current"], POINTS["target"]))
