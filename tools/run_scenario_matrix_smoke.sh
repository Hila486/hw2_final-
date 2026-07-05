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

make_lidar_configs() {
  cat > "$TMP/lidar_zero_fov.yaml" <<'YAML'
lidar_config:
  z_min_cm: 20
  z_max_cm: 120
  d_cm: 2.5
  fov_circles: 0
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
}

make_lidar_configs

# --------------------------------------------------------------------
# Part 1: valid matrix.
# Same valid simulation, same valid mission, different lidar configs.
# This checks lidar config variety without mixing incompatible maps.
# --------------------------------------------------------------------
cat > "$TMP/valid_lidar_matrix.yaml" <<YAML
simulation_compositions:
  simulations:
    - simulation_config: "$ROOT/test_configs/simulation_config.yaml"
      mission_configs:
        - "$ROOT/test_configs/mission_config.yaml"

  drone_configs:
    - "$ROOT/test_configs/drone_config.yaml"

  lidar_configs:
    - "$TMP/lidar_zero_fov.yaml"
    - "$TMP/lidar_center_beam.yaml"
    - "$TMP/lidar_short_range.yaml"
    - "$TMP/lidar_wide.yaml"
YAML

VALID_OUT="$TMP/valid_out"
mkdir -p "$VALID_OUT"

"$SIM" "$TMP/valid_lidar_matrix.yaml" "$VALID_OUT" >/dev/null

[[ -f "$VALID_OUT/simulation_output.yaml" ]] || fail "Missing valid simulation_output.yaml"
[[ -d "$VALID_OUT/output_results" ]] || fail "Missing valid output_results"

grep -q "total_runs: 4" "$VALID_OUT/simulation_output.yaml" || {
  cat "$VALID_OUT/simulation_output.yaml"
  fail "Expected total_runs: 4 for valid lidar matrix"
}

grep -q "error_runs: 0" "$VALID_OUT/simulation_output.yaml" || {
  cat "$VALID_OUT/simulation_output.yaml"
  fail "Expected error_runs: 0 for valid lidar matrix"
}

valid_maps="$(find "$VALID_OUT/output_results" -name output_map.npy | wc -l | tr -d ' ')"
[[ "$valid_maps" == "4" ]] || fail "Expected 4 output maps for valid lidar matrix, got $valid_maps"

pass "valid lidar matrix produced 4 successful runs"

# --------------------------------------------------------------------
# Part 2: error-edge matrix.
# These scenarios are intentionally questionable/invalid.
# The requirement is not that they score well, but that the simulator
# reports errors and continues instead of crashing.
# --------------------------------------------------------------------
cat > "$TMP/error_edge_matrix.yaml" <<YAML
simulation_compositions:
  simulations:
    - simulation_config: "$ROOT/test_configs/simulation_config_single_x2.yaml"
      mission_configs:
        - "$ROOT/test_configs/mission_config.yaml"

    - simulation_config: "$ROOT/test_configs/simulation_config_single_x4.yaml"
      mission_configs:
        - "$ROOT/test_configs/mission_config.yaml"

    - simulation_config: "$ROOT/test_configs/simulation_config_five_voxels.yaml"
      mission_configs:
        - "$ROOT/test_configs/mission_config.yaml"

    - simulation_config: "$ROOT/test_configs/simulation_config_two_planes.yaml"
      mission_configs:
        - "$ROOT/test_configs/mission_config.yaml"

  drone_configs:
    - "$ROOT/test_configs/drone_config.yaml"

  lidar_configs:
    - "$TMP/lidar_center_beam.yaml"
YAML

ERROR_OUT="$TMP/error_out"
mkdir -p "$ERROR_OUT"

# This command should not crash even though some runs are expected to fail.
"$SIM" "$TMP/error_edge_matrix.yaml" "$ERROR_OUT" >/dev/null 2>"$TMP/error_stderr.log" || {
  cat "$TMP/error_stderr.log"
  fail "Simulator process crashed on error-edge matrix"
}

[[ -f "$ERROR_OUT/simulation_output.yaml" ]] || fail "Missing error-edge simulation_output.yaml"
[[ -d "$ERROR_OUT/output_results" ]] || fail "Missing error-edge output_results"

grep -q "total_runs: 4" "$ERROR_OUT/simulation_output.yaml" || {
  cat "$ERROR_OUT/simulation_output.yaml"
  fail "Expected total_runs: 4 for error-edge matrix"
}

grep -q "error_runs:" "$ERROR_OUT/simulation_output.yaml" || {
  cat "$ERROR_OUT/simulation_output.yaml"
  fail "Expected error_runs field in error-edge report"
}

grep -q "score: -1" "$ERROR_OUT/simulation_output.yaml" || {
  cat "$ERROR_OUT/simulation_output.yaml"
  fail "Expected at least one failed scenario with score -1"
}

pass "error-edge matrix reports failed scenarios without crashing"

echo "[OK] scenario matrix smoke tests passed"
