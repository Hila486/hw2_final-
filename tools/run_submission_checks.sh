#!/usr/bin/env bash
set -euo pipefail

echo "[1/4] Build"
cmake --build --preset default

echo "[2/4] GTest"
./build/drone_mapper_simulation_test

echo "[3/4] Default behavior smoke"
./tools/run_default_behavior_smoke.sh

echo "[4/4] Main simulation smoke"
rm -rf run_output
mkdir -p run_output
./build/drone_mapper_simulation test_configs/simulation_compositions.yaml run_output >/dev/null

test -f run_output/simulation_output.yaml
test -d run_output/output_results
grep -q "error_runs: 0" run_output/simulation_output.yaml

echo "[OK] submission checks passed"
