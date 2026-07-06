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
D, H, W = 6, 8, 8

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

empty = bytearray(D * H * W)
write_npy(tmp / "empty.npy", empty)

column = bytearray(D * H * W)
for z in range(D):
    column[idx(z, 5, 5)] = 1
write_npy(tmp / "column.npy", column)

wall = bytearray(D * H * W)
for z in range(D):
    for y in range(H):
        wall[idx(z, y, 4)] = 1
for z in range(2, 5):
    for y in range(2, 5):
        wall[idx(z, y, 4)] = 0
write_npy(tmp / "wall_with_gap.npy", wall)

blocked = bytearray(D * H * W)
blocked[idx(2, 2, 2)] = 1
write_npy(tmp / "start_blocked.npy", blocked)
PY

run_case() {
  local name="$1"
  local map="$2"
  local drone_dim="$3"
  local lidar_zmax="$4"
  local fov="$5"
  local xmin="$6"
  local xmax="$7"
  local ymin="$8"
  local ymax="$9"
  local zmin="${10}"
  local zmax="${11}"
  local factor="${12}"
  local expect="${13}"

  CASE="$TMP/$name"
  mkdir -p "$CASE/out"

  cat > "$CASE/simulation.yaml" <<YAML
simulation_config:
  map_filename: "$map"
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: 25
    y_cm: 25
    height_cm: 25
  initial_angle_deg: 0
YAML

  cat > "$CASE/mission.yaml" <<YAML
mission_config:
  max_steps: 6000
  boundaries:
    x_boundary:
      min_cm: $xmin
      max_cm: $xmax
    y_boundary:
      min_cm: $ymin
      max_cm: $ymax
    height_boundary:
      min_cm: $zmin
      max_cm: $zmax
  gps_resolution_cm: 10
  output_mapping_resolution_factor: $factor
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

  if [[ "$expect" == "success" ]]; then
    grep -q "error_runs: 0" "$CASE/out/simulation_output.yaml" || {
      cat "$CASE/out/simulation_output.yaml"
      fail "$name expected zero error runs"
    }
    find "$CASE/out/output_results" -name output_map.npy | grep -q . || fail "$name expected output_map.npy"
    ok "$name"
  else
    if grep -q "error_runs: 0" "$CASE/out/simulation_output.yaml"; then
      cat "$CASE/out/simulation_output.yaml"
      fail "$name expected an error run"
    fi
    ok "$name reported expected error"
  fi
}

run_case empty_small_drone "$TMP/empty.npy" 10 80 3 0 80 0 80 0 60 1 success
run_case empty_large_drone "$TMP/empty.npy" 30 80 3 0 80 0 80 0 60 1 success
run_case empty_narrow_mission "$TMP/empty.npy" 10 80 3 10 70 10 70 10 50 1 success
run_case empty_resolution_factor_2 "$TMP/empty.npy" 10 80 3 0 80 0 80 0 60 2 success
run_case column_short_lidar "$TMP/column.npy" 10 40 1 0 80 0 80 0 60 1 success
run_case wall_with_gap "$TMP/wall_with_gap.npy" 10 80 3 0 80 0 80 0 60 1 success
run_case start_blocked "$TMP/start_blocked.npy" 10 80 3 0 80 0 80 0 60 1 error

ok "config variation smoke tests passed"
