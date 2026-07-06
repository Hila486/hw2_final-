#!/usr/bin/env bash
set -euo pipefail

ROOT="$(pwd)"
SIM="$ROOT/build/drone_mapper_simulation"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

fail() {
  echo "[FAIL] $*" >&2
  exit 1
}

ok() {
  echo "[OK] $*"
}

python3 - "$TMP" <<'PY'
from pathlib import Path
import struct
import sys

tmp = Path(sys.argv[1])
D, H, W = 8, 12, 12

def idx(z, y, x):
    return (z * H + y) * W + x

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

# 1. Sparse pillars
arr = empty()
for z in range(D):
    for y, x in [(3,3), (3,8), (8,3), (8,8)]:
        arr[idx(z, y, x)] = 1
write_npy(tmp / "pillars.npy", arr)

# 2. Two rooms with doorway in middle wall
arr = empty()
for z in range(D):
    for y in range(H):
        arr[idx(z, y, 6)] = 1
for z in range(2, 6):
    for y in range(5, 8):
        arr[idx(z, y, 6)] = 0
write_npy(tmp / "two_rooms_door.npy", arr)

# 3. Ceiling slab obstacle
arr = empty()
for y in range(2, 10):
    for x in range(2, 10):
        arr[idx(6, y, x)] = 1
write_npy(tmp / "ceiling_slab.npy", arr)

# 4. Floor bump / low obstacle
arr = empty()
for y in range(4, 8):
    for x in range(4, 8):
        arr[idx(1, y, x)] = 1
write_npy(tmp / "floor_bump.npy", arr)

# 5. Corridor walls with an open passage
arr = empty()
for z in range(D):
    for x in range(W):
        arr[idx(z, 3, x)] = 1
        arr[idx(z, 8, x)] = 1
for z in range(D):
    for x in range(1, 11):
        arr[idx(z, 3, x)] = 0
        arr[idx(z, 8, x)] = 0
write_npy(tmp / "corridor.npy", arr)
PY

run_case() {
  local name="$1"
  local map="$2"
  local drone_dim="$3"
  local lidar_zmax="$4"
  local fov="$5"
  local start_x="${6:-25}"
  local start_y="${7:-25}"
  local start_z="${8:-25}"

  CASE="$TMP/$name"
  mkdir -p "$CASE/out"

  cat > "$CASE/simulation.yaml" <<YAML
simulation_config:
  map_filename: "$map"
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: $start_x
    y_cm: $start_y
    height_cm: $start_z
  initial_angle_deg: 0
YAML

  cat > "$CASE/mission.yaml" <<'YAML'
mission_config:
  max_steps: 12000
  boundaries:
    x_boundary:
      min_cm: 0
      max_cm: 120
    y_boundary:
      min_cm: 0
      max_cm: 120
    height_boundary:
      min_cm: 0
      max_cm: 80
  gps_resolution_cm: 10
  output_mapping_resolution_factor: 1
YAML

  cat > "$CASE/drone.yaml" <<YAML
drone_config:
  dimensions_cm: $drone_dim
  max_rotate_deg: 45
  max_advance_cm: 100
  max_elevate_cm: 100
YAML

  cat > "$CASE/lidar.yaml" <<YAML
lidar_config:
  z_min_cm: 10
  z_max_cm: $lidar_zmax
  d_cm: 2
  fov_circles: $fov
YAML

  cat > "$CASE/composition.yaml" <<YAML
simulation_compositions:
  simulations:
    - simulation_config: "$CASE/simulation.yaml"
      mission_configs:
        - "$CASE/mission.yaml"
  drone_configs:
    - "$CASE/drone.yaml"
  lidar_configs:
    - "$CASE/lidar.yaml"
YAML

  "$SIM" "$CASE/composition.yaml" "$CASE/out" >/dev/null 2>/dev/null || true

  test -f "$CASE/out/simulation_output.yaml" || fail "$name did not create simulation_output.yaml"

  grep -q "total_runs: 1" "$CASE/out/simulation_output.yaml" || {
    cat "$CASE/out/simulation_output.yaml"
    fail "$name expected total_runs: 1"
  }

  grep -q "error_runs: 0" "$CASE/out/simulation_output.yaml" || {
    cat "$CASE/out/simulation_output.yaml"
    fail "$name expected error_runs: 0"
  }

  find "$CASE/out/output_results" -name output_map.npy | grep -q . || {
    cat "$CASE/out/simulation_output.yaml"
    fail "$name expected output_map.npy"
  }

  status="$(grep -m1 "              status:" "$CASE/out/simulation_output.yaml" | awk '{print $2}')"
  score="$(grep -m1 "              score:" "$CASE/out/simulation_output.yaml" | awk '{print $2}')"
  echo "[OK] $name status=$status score=$score"
}

run_case pillars "$TMP/pillars.npy" 10 100 4
run_case two_rooms_door "$TMP/two_rooms_door.npy" 10 100 4
run_case ceiling_slab "$TMP/ceiling_slab.npy" 10 120 5
run_case floor_bump "$TMP/floor_bump.npy" 10 100 4
run_case corridor "$TMP/corridor.npy" 10 100 4

# Same maps with larger drone and shorter lidar for extra coverage
run_case pillars_large_drone "$TMP/pillars.npy" 30 100 4 55 55 35
run_case two_rooms_short_lidar "$TMP/two_rooms_door.npy" 10 50 2

ok "more map smoke tests passed"
