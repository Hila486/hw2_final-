#!/usr/bin/env python3
import ast
import math
import struct
from collections import deque, Counter

MAP = "data_maps/benchmark_map_binary.npy"

RES_CM = 10
DRONE_RADIUS_CM = 5

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

def cell_from_position(shape, x_cm, y_cm, z_cm):
    depth, height, width = shape
    x = math.floor(x_cm / RES_CM)
    y = math.floor(y_cm / RES_CM)
    z = math.floor(z_cm / RES_CM)

    if not (0 <= z < depth and 0 <= y < height and 0 <= x < width):
        return None

    return z, y, x

def at_position(shape, data, x_cm, y_cm, z_cm):
    cell = cell_from_position(shape, x_cm, y_cm, z_cm)
    if cell is None:
        return 1
    z, y, x = cell
    return data[idx(shape, z, y, x)]

def legal_center(shape, data, x_cm, y_cm, z_cm):
    # Conservative sphere check: sample points inside the drone radius.
    # This imitates the idea of MappingAlgorithmImpl::isLegalDroneCenter,
    # but uses the real hidden map instead of the output map.
    r = DRONE_RADIUS_CM
    step = 1

    for dz in range(-r, r + 1, step):
        for dy in range(-r, r + 1, step):
            for dx in range(-r, r + 1, step):
                if dx * dx + dy * dy + dz * dz > r * r:
                    continue

                if at_position(shape, data, x_cm + dx, y_cm + dy, z_cm + dz) != 0:
                    return False

    return True

def main():
    shape, data = load_npy(MAP)
    depth, height, width = shape

    print("shape:", shape)
    print("resolution cm:", RES_CM)
    print("drone radius cm:", DRONE_RADIUS_CM)
    print("start:", (START_Z_CM, START_Y_CM, START_X_CM))

    start_legal = legal_center(shape, data, START_X_CM, START_Y_CM, START_Z_CM)
    print("start legal for drone body:", start_legal)

    all_centers = []
    legal = set()

    # Centers compatible with this map/grid: 5, 15, 25, ...
    for z in range(5, depth * RES_CM, RES_CM):
        for y in range(5, height * RES_CM, RES_CM):
            for x in range(5, width * RES_CM, RES_CM):
                center = (z, y, x)
                all_centers.append(center)
                if legal_center(shape, data, x, y, z):
                    legal.add(center)

    print("all possible center positions:", len(all_centers))
    print("legal center positions:", len(legal))

    start = (START_Z_CM, START_Y_CM, START_X_CM)
    if start not in legal:
        print("Start is not legal under this conservative body check.")
        print("This may mean the check is stricter than the simulator, or the start is too close to a wall.")
        return

    q = deque([start])
    seen = {start}

    dirs = [
        ( RES_CM, 0, 0), (-RES_CM, 0, 0),
        (0,  RES_CM, 0), (0, -RES_CM, 0),
        (0, 0,  RES_CM), (0, 0, -RES_CM),
    ]

    while q:
        z, y, x = q.popleft()
        for dz, dy, dx in dirs:
            nxt = (z + dz, y + dy, x + dx)
            if nxt in seen:
                continue
            if nxt not in legal:
                continue
            seen.add(nxt)
            q.append(nxt)

    print("reachable legal center positions:", len(seen))
    print("reachable percent of legal centers:", f"{100 * len(seen) / max(1, len(legal)):.2f}%")

    per_z_legal = Counter(z for z, y, x in legal)
    per_z_seen = Counter(z for z, y, x in seen)

    print()
    print("per z: legal vs reachable legal centers")
    for z in range(5, depth * RES_CM, RES_CM):
        print(f"z_cm={z:3d}: legal={per_z_legal[z]:4d}, reachable={per_z_seen[z]:4d}")

if __name__ == "__main__":
    main()
