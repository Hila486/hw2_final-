#!/usr/bin/env bash
set -euo pipefail

ROOT="$(pwd)"
SIM="$ROOT/build/drone_mapper_simulation"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

python3 - "$TMP" <<'PY'
from pathlib import Path
import struct
import sys

tmp = Path(sys.argv[1])
D, H, W = 5, 11, 11

def index(z, y, x):
    return ((z * H) + y) * W + x

def write_npy(path, data):
    header = "{'descr': '|u1', 'fortran_order': False, 'shape': (%d, %d, %d), }" % (D, H, W)
    hb = header.encode("latin1")
    pad = 16 - ((10 + len(hb) + 1) % 16)
    hb += b" " * pad + b"\n"
    with open(path, "wb") as f:
        f.write(b"\x93NUMPY")
        f.write(bytes([1, 0]))
        f.write(struct.pack("<H", len(hb)))
        f.write(hb)
        f.write(bytes(data))

def empty():
    return bytearray(D * H * W)

arr = empty()
write_npy(tmp / "empty.npy", arr)

arr = empty()
arr[index(2, 8, 8)] = 1
write_npy(tmp / "single_far_obstacle.npy", arr)

arr = empty()
for z in range(D):
    for x in range(W):
        arr[index(z, 0, x)] = 1
        arr[index(z, H - 1, x)] = 1
    for y in range(H):
        arr[index(z, y, 0)] = 1
        arr[index(z, y, W - 1)] = 1
write_npy(tmp / "border_walls.npy", arr)

arr = empty()
for z in range(D):
    for y, x in [(2, 2), (2, 8), (8, 2), (8, 8)]:
        arr[index(z, y, x)] = 1
write_npy(tmp / "corner_columns.npy", arr)
PY

cat > "$TMP/drone.yaml" <<'YAML'
drone_config:
  dimensions_cm: 10
  max_rotate_deg: 90
  max_advance_cm: 10
  max_elevate_cm: 10
YAML

cat > "$TMP/lidar_wide.yaml" <<'YAML'
lidar_config:
  z_min_cm: 20
  z_max_cm: 120
  d_cm: 2.5
  fov_circles: 5
YAML

cat > "$TMP/mission.yaml" <<'YAML'
mission_config:
  max_steps: 10000
  boundaries:
    x_boundary:
      min_cm: 0
      max_cm: 110
    y_boundary:
      min_cm: 0
      max_cm: 110
    height_boundary:
      min_cm: 0
      max_cm: 50
  gps_resolution_cm: 10
  output_mapping_resolution_factor: 1
YAML

run_case() {
  local name="$1"
  local map="$2"

  cat > "$TMP/simulation.yaml" <<YAML
simulation_config:
  map_filename: "$map"
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: 55
    y_cm: 55
    height_cm: 25
  initial_angle_deg: 0
YAML

  cat > "$TMP/composition.yaml" <<YAML
simulation_compositions:
  simulations:
    - simulation_config: "$TMP/simulation.yaml"
      mission_configs:
        - "$TMP/mission.yaml"
  drone_configs:
    - "$TMP/drone.yaml"
  lidar_configs:
    - "$TMP/lidar_wide.yaml"
YAML

  OUT="$TMP/out_$name"
  mkdir -p "$OUT"
  "$SIM" "$TMP/composition.yaml" "$OUT" >/dev/null 2>/dev/null || true

  STATUS="$(grep -m1 "              status:" "$OUT/simulation_output.yaml" | awk '{print $2}')"
  STEPS="$(grep -m1 "              steps:" "$OUT/simulation_output.yaml" | awk '{print $2}')"
  SCORE="$(grep -m1 "              score:" "$OUT/simulation_output.yaml" | awk '{print $2}')"

  echo "$name -> status=$STATUS steps=$STEPS score=$SCORE"
}

run_case "empty" "$TMP/empty.npy"
run_case "single_far_obstacle" "$TMP/single_far_obstacle.npy"
run_case "border_walls" "$TMP/border_walls.npy"
run_case "corner_columns" "$TMP/corner_columns.npy"
