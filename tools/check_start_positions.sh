#!/usr/bin/env bash
set -euo pipefail

ROOT="$(pwd)"
SIM="$ROOT/build/drone_mapper_simulation"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

cat > "$TMP/mission.yaml" <<'YAML'
mission_config:
  max_steps: 10000
  boundaries:
    x_boundary:
      min_cm: 0
      max_cm: 300
    y_boundary:
      min_cm: 0
      max_cm: 300
    height_boundary:
      min_cm: 0
      max_cm: 290
  gps_resolution_cm: 10
  output_mapping_resolution_factor: 1
YAML

make_and_run() {
  local name="$1"
  local x="$2"
  local y="$3"
  local z="$4"

  cat > "$TMP/simulation_$name.yaml" <<YAML
simulation_config:
  map_filename: "$ROOT/data_maps/benchmark_map_binary.npy"
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: $x
    y_cm: $y
    height_cm: $z
  initial_angle_deg: 0
YAML

  cat > "$TMP/composition_$name.yaml" <<YAML
simulation_compositions:
  simulations:
    - simulation_config: "$TMP/simulation_$name.yaml"
      mission_configs:
        - "$TMP/mission.yaml"
  drone_configs:
    - "$ROOT/test_configs/drone_config.yaml"
  lidar_configs:
    - "$ROOT/test_configs/lidar_config.yaml"
YAML

  OUT="$TMP/out_$name"
  mkdir -p "$OUT"

  "$SIM" "$TMP/composition_$name.yaml" "$OUT" >/dev/null 2>/dev/null || true

  STATUS="$(grep -m1 "              status:" "$OUT/simulation_output.yaml" | awk '{print $2}')"
  STEPS="$(grep -m1 "              steps:" "$OUT/simulation_output.yaml" | awk '{print $2}')"
  SCORE="$(grep -m1 "              score:" "$OUT/simulation_output.yaml" | awk '{print $2}')"

  MAPPED="$("$ROOT/tools/analyze_maps.py" "$ROOT/data_maps/benchmark_map_binary.npy" "$OUT/output_results/run_0000_benchmark_map_binary/output_map.npy" | grep "mapped output cells")"

  echo "$name start=($x,$y,$z) -> status=$STATUS steps=$STEPS score=$SCORE | $MAPPED"
}

make_and_run "original_low" 225 225 5
make_and_run "center_low" 155 155 5
make_and_run "center_mid" 155 155 145
make_and_run "center_high" 155 155 225
make_and_run "right_low" 225 55 25
make_and_run "upper_open" 225 225 225
