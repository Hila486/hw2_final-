# Drone Mapper Simulation — Assignment 2
Students:
Name: Shalev Cohen
ID:207610130

Name: Hila Regev Shabath
ID: 215664087


A simulator for an autonomous mapping drone. The simulator runs a batch of
scenarios (the Cartesian product of simulations, missions, drones and lidars),
drives a mapping algorithm through a hidden voxel world using mock GPS / movement
/ lidar, scores each produced map against the hidden ground-truth map, and writes
a hierarchical YAML report plus per-run output maps and error logs.

The hidden input map is used **only** by the simulator/mock-world components
(MockLidar, physical/collision validation, and final scoring). The mapping
algorithm and drone-control planning never read it — they plan only from the map
the drone discovers.

## Project Structure

```text
include/drone_mapper/   Public interfaces, data types, and component classes
src/                    Component implementations and executable entry points
tests/components/       GTest/GMock component tests (one suite per component)
tests/integration/      GTest/GMock end-to-end integration tests
tests/support/          Shared test helpers and mock doubles
data_maps/              Example NumPy (.npy) voxel maps
test_configs/           Example YAML configs and compositions
tools/                  Helper scripts (e.g. inspect_npy_maps.py)
CMakeLists.txt          CMake build configuration
vcpkg.json              Dependency list (mp-units, tinynpy, yaml-cpp, gtest)
```

## Building

```bash
cmake --preset default
cmake --build --preset default
```

Build targets:

```text
drone_mapper          Core static library
drone_mapper_simulation        The simulator executable
maps_comparison                Standalone map-comparison utility
drone_mapper_simulation_test   GTest/GMock test executable
```

## Running the simulator

```bash
./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]
```

- `<simulation.yaml>` — path to the composition YAML. If omitted, the program
  looks for `simulation.yaml` in the current working directory. A bare filename
  or relative path is resolved against the current working directory; an absolute
  path is used as-is.
- `<output_path>` — directory for the results. If omitted, the current working
  directory is used. Existing files are overwritten.

Example:

```bash
rm -rf run_output && mkdir -p run_output
./build/drone_mapper_simulation test_configs/simulation_compositions.yaml run_output
```

### Composition (Cartesian product)

For each simulation group, the simulator runs every mission belonging to that
simulation, crossed with every drone config and every lidar config:

```text
runs = Σ_simulations ( missions(simulation) × drones × lidars )
```

A failure in a single run never aborts the batch: the error is logged
immediately, that run is scored `-1`, and the batch continues.

## Output format

The simulator writes the following into `<output_path>`:

```text
<output_path>/
  simulation_output.yaml
  output_results/
    errors.log                              # batch-level errors (create/run failures)
    run_XXXX_<map_stem>/
      output_map.npy                        # the drone's discovered map (.npy, int32)
      error.log                             # written only when this run errors / logs
    run_YYYY_<map_stem>/
      ...
```

- Each run gets its own folder `run_<4-digit-index>_<input-map-stem>`.
- `output_map.npy` is a row-major `(depth, height, width) = (z, y, x)` int32 array
  using the voxel encoding below.
- Per-run problems are written to that run's `error.log`; problems that prevent a
  run from being created/executed are appended to `output_results/errors.log`.
  All errors are logged immediately when they occur.

### Voxel encoding (.npy values)

```text
-3  PotentiallyOccupied   (uncertain: a too-close lidar return)
-2  OutOfBounds
-1  Unmapped              (never observed)
 0  Empty
 1  Occupied
```

Input/hidden maps must contain only `0` and `1`; any other value is rejected with
a clean error and that run is scored `-1`.

### `simulation_output.yaml`

Results are organized hierarchically (`simulations → missions → runs`):

```yaml
score_report:
  composition_file: simulation_compositions.yaml
  generated_at_utc: "2026-06-29T14:10:25Z"
  metric: output_map_accuracy
  score_range:
    min: 0
    max: 100
  error_score: -1
  summary:
    total_runs: 2
    scored_runs: 2
    error_runs: 0
    average_score: 76.8
    min_score: 76.8
    max_score: 76.8
  simulations:
    - simulation_config: test_configs/simulation_config.yaml
      missions:
        - mission_config: test_configs/mission_config.yaml
          resolution_cm: 10                       # the actual output resolution
          resolution_request_status: ACCEPTED      # ACCEPTED | IGNORED | IGNORED_TOO_SMALL
          runs:
            - drone_config: test_configs/drone_config.yaml
              lidar_config: test_configs/lidar_config.yaml
              status: max_steps                    # completed | max_steps | error
              steps: 50
              score: 76.8                          # 0..100, or -1 on error
              # error_ref: { code: DRONE_HITS_OBSTACLE }   # present only on error
```

Notes:

- `score` is the accuracy (0–100) of the run's output map vs. the hidden map; an
  errored run scores `-1` and carries an `error_ref.code`.
- `summary` aggregates only successfully scored runs (errored runs are counted in
  `error_runs` and excluded from the averages).
- `resolution_request_status` reflects `output_mapping_resolution_factor`:
  missing/`1` → `ACCEPTED`; `> 1` → `IGNORED` (default resolution kept; producing
  a coarser map is an optional bonus); `< 1` → `IGNORED_TOO_SMALL` (logged).

## Maps comparison utility

```bash
./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```

- `origin_map` / `target_map` — `.npy` file names (with or without a path).
- `comparison_config` — optional YAML giving each map's resolution, offset and
  boundaries (see `comparison_config` in Assignment 2). If omitted, both maps are
  assumed to share the same offset, boundaries and resolution.
- Prints **only** the score (a float in `[0, 100]`) to stdout. On error it prints
  `-1` to stdout and a descriptive message to stderr.

## Running the tests

The test executable bundles the component tests (`tests/components/`) and the
integration tests (`tests/integration/`).

```bash
./build/drone_mapper_simulation_test                                  # everything
./build/drone_mapper_simulation_test --gtest_filter=Integration.*     # integration only
./build/drone_mapper_simulation_test --gtest_filter=SimulationManager.*
./build/drone_mapper_simulation_test --gtest_filter=SimulationRun.*    # also covers MockGPS/MockMovement
./build/drone_mapper_simulation_test --gtest_filter=MissionControl.*
./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
./build/drone_mapper_simulation_test --gtest_filter=MockLidar.*
./build/drone_mapper_simulation_test --gtest_filter=MapsComparison.*
```

## Inspecting maps

```bash
python3 tools/inspect_npy_maps.py data_maps/five_voxels_y4_pattern.npy
python3 tools/inspect_npy_maps.py run_output/output_results/*/output_map.npy
```
