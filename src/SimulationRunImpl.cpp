#include <drone_mapper/SimulationRunImpl.h>

#include <drone_mapper/MapsComparison.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_mapper {

namespace {

[[nodiscard]] bool hasMissionError(const types::MissionRunResult& mission_result) {
    return mission_result.status == types::MissionRunStatus::Error;
}

/*
 * Resolution request policy (Assignment 2):
 * - missing factor defaults to 1 -> the requested resolution equals the default
 *   the simulation produces, so it is ACCEPTED.
 * - factor > 1 requests a coarser output resolution. Producing a different
 *   resolution is an optional bonus; the non-bonus simulator keeps the default
 *   map resolution and reports the request as IGNORED.
 * - factor < 1 is invalid (finer than the supported/default resolution) and is
 *   reported as IGNORED_TOO_SMALL with an error-log line.
 */
[[nodiscard]] types::ResolutionRequestStatus classifyResolutionRequest(double factor) {
    if (factor < 1.0) {
        return types::ResolutionRequestStatus::IgnoredTooSmall;
    }
    if (std::abs(factor - 1.0) < 1.0e-9) {
        return types::ResolutionRequestStatus::Accepted;
    }
    return types::ResolutionRequestStatus::Ignored;
}

// Invalid resolution factors must be logged immediately. Appended (not
// truncated) so any preflight error log written by mission control survives.
void logResolutionTooSmall(const std::filesystem::path& output_map_file, double factor) {
    const std::filesystem::path run_folder = output_map_file.parent_path();
    try {
        if (!run_folder.empty()) {
            std::filesystem::create_directories(run_folder);
        }
        std::ofstream log{run_folder / "error.log", std::ios::app};
        if (log) {
            log << "output_mapping_resolution_factor=" << factor
                << " is < 1; request ignored as too small, using default map resolution.\n";
        }
    } catch (...) {
        // Logging must never throw.
    }
}

} // namespace

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      drone_control_(std::move(drone_control)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)) {
    if (!hidden_map_ ||
        !output_map_ ||
        !gps_ ||
        !movement_ ||
        !lidar_ ||
        !mapping_algorithm_ ||
        !drone_control_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    const double resolution_factor = mission_config_.output_mapping_resolution_factor;
    const types::ResolutionRequestStatus resolution_status =
        classifyResolutionRequest(resolution_factor);

    types::MissionRunResult mission_result = mission_control_->runMission(); // runs the mission

    if (resolution_status == types::ResolutionRequestStatus::IgnoredTooSmall) {
        logResolutionTooSmall(output_map_file_, resolution_factor);
    }

    // compares the output map to the hidden map and calculates the mission score
    double mission_score = -1.0;
    if (!hasMissionError(mission_result)) {
        mission_score = 0.0;
        const std::vector<IMap3D*> target_maps{output_map_.get()};
        const std::vector<double> scores = MapsComparison::compare(*hidden_map_, target_maps);
        if (!scores.empty()) {
            mission_score = scores.front();
        }
    }
    // returns the simulation result
    return types::SimulationResult{
        simulation_config_,
        mission_config_,
        resolution_status,
        {std::move(mission_result)},
        output_map_file_,
        output_map_->getMapConfig(),
        mission_score,
    };
}

} // namespace drone_mapper
