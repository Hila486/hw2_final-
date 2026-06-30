#include <drone_mapper/SimulationManager.h>

#include <chrono>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_mapper {

namespace {

[[nodiscard]] std::string utcTimestamp() {
    const std::time_t now =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm_utc{};
    gmtime_r(&now, &tm_utc);
    std::ostringstream stream;
    stream << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
    return stream.str();
}

// Errors must be written immediately (not deferred), so each call appends a
// single line to a batch-level log under the output path and also echoes to
// stderr. Logging itself must never throw and abort the batch.
void logBatchError(const std::filesystem::path& output_path, const std::string& message) {
    try {
        const std::filesystem::path log_dir = output_path / "output_results";
        std::filesystem::create_directories(log_dir);
        std::ofstream log{log_dir / "errors.log", std::ios::app};
        if (log) {
            log << utcTimestamp() << " " << message << '\n';
        }
    } catch (...) {
        // Swallow secondary logging failures.
    }
    std::cerr << "Simulation run error: " << message << '\n';
}

// A run that cannot even be created/executed still needs a placeholder result
// so the batch can continue and the failure is reported with score -1.
[[nodiscard]] types::SimulationResult makeFailedResult(
    const types::SimulationConfigData& simulation,
    const types::MissionConfigData& mission,
    std::string message) {
    types::SimulationResult result;
    result.simulation_config = simulation;
    result.mission_config = mission;
    result.resolution_request_status = types::ResolutionRequestStatus::Ignored;
    result.mission_results = {
        types::MissionRunResult{
            types::MissionRunStatus::Error,
            0,
            {types::ErrorRef{"RUN_FAILED", std::move(message)}},
        },
    };
    result.mission_score = -1.0;
    return result;
}

} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;

    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        for (const types::MissionConfigData& mission : missions) {
            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    // A failure in one run (e.g. an unreadable map) must not
                    // abort the whole batch: log it, score it -1, and continue.
                    try {
                        std::unique_ptr<ISimulationRun> run =
                            run_factory_->create(simulation, mission, drone, lidar, output_path);
                        if (!run) {
                            throw std::runtime_error("Run factory returned a null simulation run.");
                        }
                        runs.push_back(run->run());
                    } catch (const std::exception& exception) {
                        logBatchError(output_path, exception.what());
                        runs.push_back(makeFailedResult(simulation, mission, exception.what()));
                    } catch (...) {
                        const std::string message = "Unknown error during simulation run.";
                        logBatchError(output_path, message);
                        runs.push_back(makeFailedResult(simulation, mission, message));
                    }
                }
            }
        }
    }

    return types::SimulationManagerReport{
        utcTimestamp(),
        "output_map_accuracy",
        std::tuple<double, double>{0.0, 100.0},
        -1,
        std::move(runs),
    };
}

} // namespace drone_mapper
