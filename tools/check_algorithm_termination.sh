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
data = bytearray(D * H * W)

header = "{'descr': '|u1', 'fortran_order': False, 'shape': (%d, %d, %d), }" % (D, H, W)
hb = header.encode("latin1")
pad = 16 - ((10 + len(hb) + 1) % 16)
hb += b" " * pad + b"\n"

with open(tmp / "empty.npy", "wb") as f:
    f.write(b"\x93NUMPY")
    f.write(bytes([1, 0]))
    f.write(struct.pack("<H", len(hb)))
    f.write(hb)
    f.write(bytes(data))
PY

cat > "$TMP/simulation.yaml" <<YAML
simulation_config:
  map_filename: "$TMP/empty.npy"
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: 55
    y_cm: 55
    height_cm: 25
  initial_angle_deg: 0
YAML

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

for STEPS in 100 250 500 1000 2000 5000 10000; do
  cat > "$TMP/mission.yaml" <<YAML
mission_config:
  max_steps: $STEPS
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

  OUT="$TMP/out_$STEPS"
  mkdir -p "$OUT"
  "$SIM" "$TMP/composition.yaml" "$OUT" >/dev/null

  STATUS="$(grep -m1 "              status:" "$OUT/simulation_output.yaml" | awk '{print $2}')"
  RUN_STEPS="$(grep -m1 "              steps:" "$OUT/simulation_output.yaml" | awk '{print $2}')"
  SCORE="$(grep -m1 "              score:" "$OUT/simulation_output.yaml" | awk '{print $2}')"

  echo "max_steps=$STEPS -> status=$STATUS steps=$RUN_STEPS score=$SCORE"
done
