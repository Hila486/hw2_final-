import ast
import struct
from pathlib import Path
from collections import Counter
import sys

def read_npy(path):
    path = Path(path)
    with path.open("rb") as f:
        magic = f.read(6)
        version = f.read(2)

        if magic != b"\x93NUMPY":
            raise ValueError(f"{path} is not a valid .npy file")

        if version == b"\x01\x00":
            header_len = int.from_bytes(f.read(2), "little")
        else:
            header_len = int.from_bytes(f.read(4), "little")

        header_text = f.read(header_len).decode("latin1")
        header = ast.literal_eval(header_text)

        descr = header["descr"]
        shape = header["shape"]

        count = 1
        for dim in shape:
            count *= dim

        dtype = descr[-2:]

        if dtype == "u1":
            fmt, size = "B", 1
        elif dtype == "i1":
            fmt, size = "b", 1
        elif dtype == "u2":
            fmt, size = "H", 2
        elif dtype == "i2":
            fmt, size = "h", 2
        elif dtype == "u4":
            fmt, size = "I", 4
        elif dtype == "i4":
            fmt, size = "i", 4
        else:
            raise ValueError(f"Unsupported dtype {descr} in {path}")

        raw = f.read(count * size)
        values = list(struct.unpack("<" + fmt * count, raw))

    return header, values

def meaning(value):
    return {
        -3: "PotentiallyOccupied",
        -2: "OutOfBounds",
        -1: "Unmapped",
         0: "Empty/free",
         1: "Occupied",
    }.get(value, "UnknownValue")

def inspect(path, show_slices=True):
    path = Path(path)
    print("\n" + "=" * 70)
    print("FILE:", path)

    header, values = read_npy(path)
    shape = header["shape"]

    print("HEADER:", header)
    print("SHAPE:", shape)
    print("TOTAL VOXELS:", len(values))

    print("\nVALUE COUNTS:")
    for value, amount in sorted(Counter(values).items()):
        print(f"  {value:>4} ({meaning(value)}): {amount}")

    if show_slices and len(shape) == 3:
        depth, height, width = shape
        print("\nSLICES:")
        idx = 0
        for z in range(depth):
            print(f"\nz = {z}")
            for y in range(height):
                row = values[idx:idx + width]
                idx += width
                print(" ".join(f"{v:3d}" for v in row))

def main():
    if len(sys.argv) > 1:
        files = [Path(arg) for arg in sys.argv[1:]]
    else:
        files = []
        files += sorted(Path("data_maps").glob("*.npy"))
        files += sorted(Path("run_output").glob("output_results/**/output_map.npy"))

    if not files:
        print("No .npy files found. Pass file paths as arguments.")
        return

    for file in files:
        inspect(file)

if __name__ == "__main__":
    main()
