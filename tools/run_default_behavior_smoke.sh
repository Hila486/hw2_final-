#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="$ROOT/build/drone_mapper_simulation"
CMP="$ROOT/build/maps_comparison"

fail() {
  echo "[FAIL] $*" >&2
  exit 1
}

pass() {
  echo "[OK] $*"
}

[[ -x "$SIM" ]] || fail "Missing executable: $SIM. Run: cmake --build --preset default"
[[ -x "$CMP" ]] || fail "Missing executable: $CMP. Run: cmake --build --preset default"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

make_abs_sim_config() {
  local dst="$1"
  cp "$ROOT/test_configs/simulation_config.yaml" "$dst"
  sed -i "s#map_filename: .*#map_filename: \"$ROOT/data_maps/benchmark_map_binary.npy\"#" "$dst"
}

make_composition() {
  local dst="$1"
  local sim_cfg="$2"
  local mission_cfg="$3"

  cat > "$dst" <<YAML
simulation_compositions:
  simulations:
    - simulation_config: "$sim_cfg"
      mission_configs:
        - "$mission_cfg"
  drone_configs:
    - "$ROOT/test_configs/drone_config.yaml"
  lidar_configs:
    - "$ROOT/test_configs/lidar_config.yaml"
YAML
}

assert_report_ok() {
  local outdir="$1"

  [[ -f "$outdir/simulation_output.yaml" ]] || fail "Missing $outdir/simulation_output.yaml"
  [[ -d "$outdir/output_results" ]] || fail "Missing $outdir/output_results"

  grep -q "error_runs: 0" "$outdir/simulation_output.yaml" || {
    cat "$outdir/simulation_output.yaml"
    fail "Expected error_runs: 0 in $outdir/simulation_output.yaml"
  }

  grep -q "status: completed" "$outdir/simulation_output.yaml" || {
    cat "$outdir/simulation_output.yaml"
    fail "Expected at least one completed run in $outdir/simulation_output.yaml"
  }
}

# Base absolute simulation config, so tests do not depend on current directory.
make_abs_sim_config "$TMP/simulation_abs.yaml"
make_composition "$TMP/composition_abs.yaml" "$TMP/simulation_abs.yaml" "$ROOT/test_configs/mission_config.yaml"

# 1. Missing initial_angle_deg should default to 0.
cp "$TMP/simulation_abs.yaml" "$TMP/simulation_no_angle.yaml"
sed -i '/initial_angle_deg/d' "$TMP/simulation_no_angle.yaml"
make_composition "$TMP/composition_no_angle.yaml" "$TMP/simulation_no_angle.yaml" "$ROOT/test_configs/mission_config.yaml"

mkdir -p "$TMP/out_no_angle"
"$SIM" "$TMP/composition_no_angle.yaml" "$TMP/out_no_angle" >/dev/null
assert_report_ok "$TMP/out_no_angle"
pass "missing initial_angle_deg defaults to 0"

# 2. Missing output_mapping_resolution_factor should default to 1.
cp "$ROOT/test_configs/mission_config.yaml" "$TMP/mission_no_resolution_factor.yaml"
sed -i '/output_mapping_resolution_factor/d' "$TMP/mission_no_resolution_factor.yaml"
make_composition "$TMP/composition_no_factor.yaml" "$TMP/simulation_abs.yaml" "$TMP/mission_no_resolution_factor.yaml"

mkdir -p "$TMP/out_no_factor"
"$SIM" "$TMP/composition_no_factor.yaml" "$TMP/out_no_factor" >/dev/null
assert_report_ok "$TMP/out_no_factor"
grep -q "resolution_request_status: ACCEPTED" "$TMP/out_no_factor/simulation_output.yaml" || {
  cat "$TMP/out_no_factor/simulation_output.yaml"
  fail "Expected missing output_mapping_resolution_factor to behave like factor=1 / ACCEPTED"
}
pass "missing output_mapping_resolution_factor defaults to 1"

# 3. Missing output path should write to current directory.
mkdir -p "$TMP/no_output_path_work"
cp "$TMP/composition_abs.yaml" "$TMP/no_output_path_work/composition.yaml"
(
  cd "$TMP/no_output_path_work"
  "$SIM" "composition.yaml" >/dev/null
)
assert_report_ok "$TMP/no_output_path_work"
pass "missing output_path writes to current directory"

# 4. Missing simulation argument should use simulation.yaml in current directory.
mkdir -p "$TMP/no_sim_arg_work"
cp "$TMP/composition_abs.yaml" "$TMP/no_sim_arg_work/simulation.yaml"
(
  cd "$TMP/no_sim_arg_work"
  "$SIM" >/dev/null
)
assert_report_ok "$TMP/no_sim_arg_work"
pass "missing simulation argument uses simulation.yaml"

# 5. Filename-only simulation path should work from current directory.
mkdir -p "$TMP/filename_only_work"
cp "$TMP/composition_abs.yaml" "$TMP/filename_only_work/composition.yaml"
mkdir -p "$TMP/filename_only_output"
(
  cd "$TMP/filename_only_work"
  "$SIM" "composition.yaml" "$TMP/filename_only_output" >/dev/null
)
assert_report_ok "$TMP/filename_only_output"
pass "filename-only simulation path works"

# 6. Relative simulation path should work under current working directory.
mkdir -p "$TMP/relative_output"
(
  cd "$ROOT"
  "$SIM" "test_configs/simulation_compositions.yaml" "$TMP/relative_output" >/dev/null
)
assert_report_ok "$TMP/relative_output"
pass "relative simulation path works"

# 7. Absolute simulation path should work.
mkdir -p "$TMP/absolute_output"
"$SIM" "$ROOT/test_configs/simulation_compositions.yaml" "$TMP/absolute_output" >/dev/null
assert_report_ok "$TMP/absolute_output"
pass "absolute simulation path works"

# 8. maps_comparison without comparison_config should assume same config.
score="$("$CMP" "$ROOT/data_maps/benchmark_map_binary.npy" "$ROOT/data_maps/benchmark_map_binary.npy")"
python3 - "$score" <<'PY'
import sys
score = float(sys.argv[1].strip())
if abs(score - 100.0) > 1e-9:
    raise SystemExit(f"Expected score 100 for identical maps without comparison_config, got {score}")
PY
pass "maps_comparison without comparison_config scores identical maps as 100"

echo "[OK] all default-behavior smoke tests passed"
