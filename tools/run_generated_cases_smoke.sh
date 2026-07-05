#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="$ROOT/build/drone_mapper_simulation"

fail() {
  echo "[FAIL] $*" >&2
  exit 1
}

pass() {
  echo "[OK] $*"
}

[[ -x "$SIM" ]] || fail "Missing executable: $SIM. Run: cmake --build --preset default"

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
    header_bytes = header.encode("latin1")
    pad_len = 16 - ((10 + len(header_bytes) + 1) % 16)
    header_bytes += b" " * pad_len + b"\n"
    with open(path, "wb") as f:
        f.write(b"\x93NUMPY")
        f.write(bytes([1, 0]))
        f.write(struct.pack("<H", len(header_bytes)))
        f.write(header_bytes)
        f.write(bytes(data))

def empty():
    return bytearray(D * H * W)

arr = empty()
write_npy(tmp / "generated_empty.npy", arr)

arr = empty()
arr[index(2, 8, 8)] = 1
write_npy(tmp / "generated_single_far_obstacle.npy", arr)

arr = empty()
for z in range(D):
    for x in range(W):
        arr[index(z, 0, x)] = 1
        arr[index(z, H - 1, x)] = 1
    for y in range(H):
        arr[index(z, y, 0)] = 1
        arr[index(z, y, W - 1)] = 1
write_npy(tmp / "generated_border_walls.npy", arr)

arr = empty()
for z in range(D):
    for y, x in [(2, 2), (2, 8), (8, 2), (8, 8)]:
        arr[index(z, y, x)] = 1
write_npy(tmp / "generated_corner_columns.npy", arr)

arr = empty()
arr[index(0, 0, 0)] = 9
write_npy(tmp / "generated_invalid_value.npy", arr)

arr = empty()
arr[index(2, 5, 5)] = 1
write_npy(tmp / "generated_start_blocked.npy", arr)
PY

cat > "$TMP/mission_generated.yaml" <<'YAML'
mission_config:
  max_steps: 250
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

cat > "$TMP/drone_small.yaml" <<'YAML'
drone_config:
  dimensions_cm: 10
  max_rotate_deg: 90
  max_advance_cm: 10
  max_elevate_cm: 10
YAML

cat > "$TMP/drone_medium.yaml" <<'YAML'
drone_config:
  dimensions_cm: 20
  max_rotate_deg: 90
  max_advance_cm: 10
  max_elevate_cm: 10
YAML

cat > "$TMP/drone_large.yaml" <<'YAML'
drone_config:
  dimensions_cm: 30
  max_rotate_deg: 90
  max_advance_cm: 10
  max_elevate_cm: 10
YAML

cat > "$TMP/lidar_center_beam.yaml" <<'YAML'
lidar_config:
  z_min_cm: 20
  z_max_cm: 120
  d_cm: 2.5
  fov_circles: 1
YAML

cat > "$TMP/lidar_short_range.yaml" <<'YAML'
lidar_config:
  z_min_cm: 20
  z_max_cm: 40
  d_cm: 2.5
  fov_circles: 3
YAML

cat > "$TMP/lidar_wide.yaml" <<'YAML'
lidar_config:
  z_min_cm: 20
  z_max_cm: 120
  d_cm: 2.5
  fov_circles: 5
YAML

make_sim_config() {
  local dst="$1"
  local map="$2"
  local x="${3:-55}"
  local y="${4:-55}"
  local z="${5:-25}"

  cat > "$dst" <<YAML
simulation_config:
  map_filename: "$map"
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: $x
    y_cm: $y
    height_cm: $z
  initial_angle_deg: 0
YAML
}

make_sim_config "$TMP/sim_empty.yaml" "$TMP/generated_empty.npy"
make_sim_config "$TMP/sim_single_far_obstacle.yaml" "$TMP/generated_single_far_obstacle.npy"
make_sim_config "$TMP/sim_border_walls.yaml" "$TMP/generated_border_walls.npy"
make_sim_config "$TMP/sim_corner_columns.yaml" "$TMP/generated_corner_columns.npy"

# --------------------------------------------------------------------
# Part 1: safe matrix.
# Empty map × drone configs × lidar configs.
# This should never produce an error.
# --------------------------------------------------------------------
cat > "$TMP/safe_empty_matrix.yaml" <<YAML
simulation_compositions:
  simulations:
    - simulation_config: "$TMP/sim_empty.yaml"
      mission_configs:
        - "$TMP/mission_generated.yaml"

  drone_configs:
    - "$TMP/drone_small.yaml"
    - "$TMP/drone_medium.yaml"
    - "$TMP/drone_large.yaml"

  lidar_configs:
    - "$TMP/lidar_center_beam.yaml"
    - "$TMP/lidar_short_range.yaml"
    - "$TMP/lidar_wide.yaml"
YAML

SAFE_OUT="$TMP/safe_out"
mkdir -p "$SAFE_OUT"

"$SIM" "$TMP/safe_empty_matrix.yaml" "$SAFE_OUT" >/dev/null

grep -q "total_runs: 9" "$SAFE_OUT/simulation_output.yaml" || {
  cat "$SAFE_OUT/simulation_output.yaml"
  fail "Expected total_runs: 9 for safe empty matrix"
}

grep -q "error_runs: 0" "$SAFE_OUT/simulation_output.yaml" || {
  cat "$SAFE_OUT/simulation_output.yaml"
  fail "Expected error_runs: 0 for safe empty matrix"
}

safe_maps="$(find "$SAFE_OUT/output_results" -name output_map.npy | wc -l | tr -d ' ')"
[[ "$safe_maps" == "9" ]] || fail "Expected 9 output maps, got $safe_maps"

pass "safe empty map matrix produced 9 successful runs"

# --------------------------------------------------------------------
# Part 2: obstacle stress matrix.
# Obstacle maps may expose weak algorithm behavior. We do not require
# zero errors. We require the simulator to continue and produce a report.
# --------------------------------------------------------------------
cat > "$TMP/obstacle_stress_matrix.yaml" <<YAML
simulation_compositions:
  simulations:
    - simulation_config: "$TMP/sim_single_far_obstacle.yaml"
      mission_configs:
        - "$TMP/mission_generated.yaml"

    - simulation_config: "$TMP/sim_border_walls.yaml"
      mission_configs:
        - "$TMP/mission_generated.yaml"

    - simulation_config: "$TMP/sim_corner_columns.yaml"
      mission_configs:
        - "$TMP/mission_generated.yaml"

  drone_configs:
    - "$TMP/drone_small.yaml"
    - "$TMP/drone_medium.yaml"
    - "$TMP/drone_large.yaml"

  lidar_configs:
    - "$TMP/lidar_center_beam.yaml"
    - "$TMP/lidar_short_range.yaml"
    - "$TMP/lidar_wide.yaml"
YAML

STRESS_OUT="$TMP/stress_out"
mkdir -p "$STRESS_OUT"

"$SIM" "$TMP/obstacle_stress_matrix.yaml" "$STRESS_OUT" >/dev/null 2>"$TMP/stress_stderr.log" || {
  cat "$TMP/stress_stderr.log"
  fail "Simulator process crashed on obstacle stress matrix"
}

grep -q "total_runs: 27" "$STRESS_OUT/simulation_output.yaml" || {
  cat "$STRESS_OUT/simulation_output.yaml"
  fail "Expected total_runs: 27 for obstacle stress matrix"
}

grep -q "scored_runs:" "$STRESS_OUT/simulation_output.yaml" || {
  cat "$STRESS_OUT/simulation_output.yaml"
  fail "Expected scored_runs field in obstacle stress matrix"
}

grep -Eq "status: (completed|max_steps|error)" "$STRESS_OUT/simulation_output.yaml" || {
  cat "$STRESS_OUT/simulation_output.yaml"
  fail "Expected run statuses in obstacle stress matrix"
}

pass "obstacle stress matrix completed without crashing"

# --------------------------------------------------------------------
# Part 3: explicit edge/error scenarios.
# These should report failures without crashing the process.
# --------------------------------------------------------------------
make_sim_config "$TMP/sim_invalid_value.yaml" "$TMP/generated_invalid_value.npy"
make_sim_config "$TMP/sim_start_blocked.yaml" "$TMP/generated_start_blocked.npy"
make_sim_config "$TMP/sim_start_out_of_bounds.yaml" "$TMP/generated_empty.npy" 1000 1000 25
make_sim_config "$TMP/sim_missing_map.yaml" "$TMP/does_not_exist.npy"

cat > "$TMP/generated_edge_matrix.yaml" <<YAML
simulation_compositions:
  simulations:
    - simulation_config: "$TMP/sim_invalid_value.yaml"
      mission_configs:
        - "$TMP/mission_generated.yaml"

    - simulation_config: "$TMP/sim_start_blocked.yaml"
      mission_configs:
        - "$TMP/mission_generated.yaml"

    - simulation_config: "$TMP/sim_start_out_of_bounds.yaml"
      mission_configs:
        - "$TMP/mission_generated.yaml"

    - simulation_config: "$TMP/sim_missing_map.yaml"
      mission_configs:
        - "$TMP/mission_generated.yaml"

  drone_configs:
    - "$TMP/drone_small.yaml"

  lidar_configs:
    - "$TMP/lidar_center_beam.yaml"
YAML

EDGE_OUT="$TMP/edge_out"
mkdir -p "$EDGE_OUT"

"$SIM" "$TMP/generated_edge_matrix.yaml" "$EDGE_OUT" >/dev/null 2>"$TMP/edge_stderr.log" || {
  cat "$TMP/edge_stderr.log"
  fail "Simulator process crashed on generated edge matrix"
}

grep -q "total_runs: 4" "$EDGE_OUT/simulation_output.yaml" || {
  cat "$EDGE_OUT/simulation_output.yaml"
  fail "Expected total_runs: 4 for generated edge matrix"
}

grep -Eq "error_runs: [1-9]" "$EDGE_OUT/simulation_output.yaml" || {
  cat "$EDGE_OUT/simulation_output.yaml"
  fail "Expected at least one error run in generated edge matrix"
}

grep -q "score: -1" "$EDGE_OUT/simulation_output.yaml" || {
  cat "$EDGE_OUT/simulation_output.yaml"
  fail "Expected at least one failed scenario with score -1"
}

pass "generated edge matrix reports failed scenarios without crashing"

echo "[OK] generated map/drone/lidar smoke tests passed"
