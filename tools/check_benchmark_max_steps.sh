#!/usr/bin/env bash
set -euo pipefail

ROOT="$(pwd)"
ORIG="test_configs/mission_config.yaml"
BACKUP="/tmp/mission_config.before_max_steps_check.yaml"

cp "$ORIG" "$BACKUP"

for STEPS in 1000 2500 5000 10000 20000 40000 80000; do
  python3 - "$STEPS" <<'PY'
from pathlib import Path
import sys

steps = sys.argv[1]
p = Path("test_configs/mission_config.yaml")
text = p.read_text()
lines = []
for line in text.splitlines():
    if line.strip().startswith("max_steps:"):
        indent = line[:len(line) - len(line.lstrip())]
        lines.append(f"{indent}max_steps: {steps}")
    else:
        lines.append(line)
p.write_text("\n".join(lines) + "\n")
PY

  rm -rf run_output
  mkdir -p run_output

  ./build/drone_mapper_simulation test_configs/simulation_compositions.yaml run_output >/dev/null 2>/dev/null || true

  STATUS="$(grep -m1 "              status:" run_output/simulation_output.yaml | awk '{print $2}')"
  RUN_STEPS="$(grep -m1 "              steps:" run_output/simulation_output.yaml | awk '{print $2}')"
  SCORE="$(grep -m1 "              score:" run_output/simulation_output.yaml | awk '{print $2}')"

  MAPPED_LINE="$(./tools/analyze_maps.py data_maps/benchmark_map_binary.npy run_output/output_results/run_0000_benchmark_map_binary/output_map.npy | grep "mapped output cells")"

  echo "max_steps=$STEPS -> status=$STATUS steps=$RUN_STEPS score=$SCORE | $MAPPED_LINE"
done

cp "$BACKUP" "$ORIG"
