#!/usr/bin/env python3
import ast
import struct
import sys
from collections import Counter

def load_npy(path):
    with open(path, "rb") as f:
        magic = f.read(6)
        if magic != b"\x93NUMPY":
            raise ValueError(f"{path}: not a .npy file")

        major, minor = f.read(2)
        if major == 1:
            header_len = struct.unpack("<H", f.read(2))[0]
        elif major == 2:
            header_len = struct.unpack("<I", f.read(4))[0]
        else:
            raise ValueError(f"{path}: unsupported npy version {major}.{minor}")

        header = f.read(header_len).decode("latin1").strip()
        meta = ast.literal_eval(header)

        shape = tuple(meta["shape"])
        descr = meta["descr"]
        fortran = meta["fortran_order"]

        if fortran:
            raise ValueError(f"{path}: Fortran-order arrays are not supported")

        raw = f.read()

    if descr in ("|u1", "<u1", "u1"):
        data = [b for b in raw]
    elif descr in ("|i1", "<i1", "i1"):
        data = [struct.unpack("b", bytes([b]))[0] for b in raw]
    elif descr in ("<i4", "i4"):
        data = list(struct.unpack("<" + "i" * (len(raw) // 4), raw))
    else:
        raise ValueError(f"{path}: unsupported dtype {descr}")

    expected = 1
    for d in shape:
        expected *= d

    if len(data) != expected:
        raise ValueError(f"{path}: expected {expected} values, got {len(data)}")

    return shape, descr, data

def idx(shape, z, y, x):
    depth, height, width = shape
    return (z * height + y) * width + x

def count_values(data):
    return dict(sorted(Counter(data).items()))

def describe(name, shape, descr, data):
    print(f"\n{name}")
    print("-" * len(name))
    print("shape:", shape)
    print("dtype:", descr)
    print("counts:", count_values(data))

    total = len(data)
    labels = {
        -3: "potential",
        -2: "out_of_bounds",
        -1: "unmapped",
         0: "empty",
         1: "occupied",
         9: "invalid_9",
    }

    counts = Counter(data)
    for value in sorted(counts):
        label = labels.get(value, f"value_{value}")
        c = counts[value]
        print(f"{label:14s} {c:8d}  {100*c/total:6.2f}%")

def compare(origin_shape, origin, target_shape, target):
    print("\nComparison quick stats")
    print("----------------------")

    if origin_shape != target_shape:
        print("Different shapes, cannot raw-compare directly.")
        return

    total = len(origin)
    mapped_indices = [i for i, v in enumerate(target) if v in (0, 1)]
    mapped_count = len(mapped_indices)

    print("mapped output cells:", mapped_count, "/", total, f"({100*mapped_count/total:.2f}%)")

    if mapped_count:
        correct = sum(1 for i in mapped_indices if origin[i] == target[i])
        print("correct among mapped:", correct, "/", mapped_count, f"({100*correct/mapped_count:.2f}%)")

    occupied_total = sum(1 for v in origin if v == 1)
    occupied_found = sum(1 for o, t in zip(origin, target) if o == 1 and t == 1)

    empty_total = sum(1 for v in origin if v == 0)
    empty_found = sum(1 for o, t in zip(origin, target) if o == 0 and t == 0)

    print("occupied found:", occupied_found, "/", occupied_total, f"({100*occupied_found/max(1, occupied_total):.2f}%)")
    print("empty found:", empty_found, "/", empty_total, f"({100*empty_found/max(1, empty_total):.2f}%)")

    false_occupied = sum(1 for o, t in zip(origin, target) if o == 0 and t == 1)
    false_empty = sum(1 for o, t in zip(origin, target) if o == 1 and t == 0)

    print("false occupied, origin empty but output occupied:", false_occupied)
    print("false empty, origin occupied but output empty:", false_empty)

    depth, height, width = origin_shape
    print("\nPer z-slice counts")
    print("------------------")
    for z in range(depth):
        indices = [idx(origin_shape, z, y, x) for y in range(height) for x in range(width)]
        o_occ = sum(1 for i in indices if origin[i] == 1)
        t_occ = sum(1 for i in indices if target[i] == 1)
        t_free = sum(1 for i in indices if target[i] == 0)
        t_unm = sum(1 for i in indices if target[i] == -1)
        print(f"z={z}: origin_occ={o_occ:4d}, output_occ={t_occ:4d}, output_free={t_free:4d}, output_unmapped={t_unm:4d}")

def ascii_slice(shape, data, z, title):
    depth, height, width = shape
    print(f"\n{title} z={z}")
    print("-" * (len(title) + 4))

    symbols = {
        -3: "?",
        -2: "X",
        -1: ".",
         0: " ",
         1: "#",
         9: "9",
    }

    for y in range(height):
        row = []
        for x in range(width):
            value = data[idx(shape, z, y, x)]
            row.append(symbols.get(value, str(value)[0]))
        print("".join(row))

def main():
    if len(sys.argv) != 3:
        print("Usage: analyze_maps.py <origin.npy> <target.npy>", file=sys.stderr)
        sys.exit(2)

    origin_shape, origin_descr, origin = load_npy(sys.argv[1])
    target_shape, target_descr, target = load_npy(sys.argv[2])

    describe("Origin/input map", origin_shape, origin_descr, origin)
    describe("Target/output map", target_shape, target_descr, target)
    compare(origin_shape, origin, target_shape, target)

    if "--ascii" in sys.argv:
        print("\nASCII slices legend")
        print("-------------------")
        print("# occupied, space empty, . unmapped, X out_of_bounds, ? potential")

        if origin_shape == target_shape:
            depth = origin_shape[0]
            for z in range(depth):
                ascii_slice(origin_shape, origin, z, "Origin")
                ascii_slice(target_shape, target, z, "Output")

if __name__ == "__main__":
    main()
