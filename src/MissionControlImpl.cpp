#include <drone_mapper/MissionControlImpl.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace drone_mapper {

namespace {

void addError(std::vector<types::ErrorRef>& errors, // adds error to the error vector
              std::string code,
              std::string message) {
    errors.push_back(types::ErrorRef{std::move(code), std::move(message)});
}

[[nodiscard]] double physicalCm(PhysicalLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double xCm(XLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double yCm(YLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] double zCm(ZLength length) {
    return length.force_numerical_value_in(cm);
}

struct BoundsCm {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    double min_z = 0.0;
    double max_z = 0.0;
};

// Tangency tolerance: touching a boundary or an occupied voxel exactly is not a
// collision. Only penetration beyond this small margin counts as a problem.
constexpr double kBoundaryEpsilonCm = 1.0e-6;

enum class PlacementProblem {
    None,
    OutOfBounds,
    Collision,
    DataError,
};

struct PlacementCheck {
    PlacementProblem problem = PlacementProblem::None;
    std::string message;
};

[[nodiscard]] BoundsCm boundsFromConfig(const types::MapConfig& config) {
    return BoundsCm{
        xCm(config.boundaries.min_x),
        xCm(config.boundaries.max_x),
        yCm(config.boundaries.min_y),
        yCm(config.boundaries.max_y),
        zCm(config.boundaries.min_height),
        zCm(config.boundaries.max_height),
    };
}

[[nodiscard]] bool sphereIntersectsVoxel(double center_x,
                                         double center_y,
                                         double center_z,
                                         double radius_cm,
                                         double min_x,
                                         double max_x,
                                         double min_y,
                                         double max_y,
                                         double min_z,
                                         double max_z) {
    const auto axisDistance = [](double center, double min_value, double max_value) {
        if (center < min_value) {
            return min_value - center;
        }
        if (center > max_value) {
            return center - max_value;
        }
        return 0.0;
    };

    const double dx = axisDistance(center_x, min_x, max_x);
    const double dy = axisDistance(center_y, min_y, max_y);
    const double dz = axisDistance(center_z, min_z, max_z);
    return dx * dx + dy * dy + dz * dz <= radius_cm * radius_cm;
}

[[nodiscard]] types::ErrorRef preflightError(const PlacementCheck& check) {
    switch (check.problem) {
        case PlacementProblem::OutOfBounds:
            return types::ErrorRef{
                "INITIAL_DRONE_OUT_OF_MAP",
                "Initial drone sphere is outside the real hidden map bounds. " + check.message};
        case PlacementProblem::Collision:
            return types::ErrorRef{
                "INITIAL_DRONE_COLLIDES_WITH_OCCUPIED_VOXEL",
                "Initial drone sphere intersects an occupied hidden-map voxel. " + check.message};
        case PlacementProblem::DataError:
        case PlacementProblem::None:
            break;
    }
    return types::ErrorRef{"INITIAL_DRONE_MAP_DATA_ERROR",
                           "Initial drone validation failed: " + check.message};
}

[[nodiscard]] types::ErrorRef inFlightError(const PlacementCheck& check) {
    switch (check.problem) {
        case PlacementProblem::OutOfBounds:
            return types::ErrorRef{"DRONE_OUT_OF_MAP",
                                   "Drone moved outside the real hidden map bounds. " + check.message};
        case PlacementProblem::Collision:
            return types::ErrorRef{"DRONE_HITS_OBSTACLE",
                                   "Drone body penetrated an occupied hidden-map voxel. " + check.message};
        case PlacementProblem::DataError:
        case PlacementProblem::None:
            break;
    }
    return types::ErrorRef{"DRONE_MAP_DATA_ERROR",
                           "Drone physical validation failed: " + check.message};
}

/*
 * Simulator-side physical validation of the drone sphere against the hidden
 * map. This is the ONLY place the hidden map is consulted for collisions; the
 * mapping algorithm and drone-control planning never see it. Used both for the
 * initial placement (preflight) and after every executed step (in flight).
 *
 * Exact tangency with a boundary or an occupied voxel is allowed; only positive
 * penetration beyond kBoundaryEpsilonCm is reported as a problem.
 */
[[nodiscard]] PlacementCheck checkDronePlacement(const IMap3D& hidden_map,
                                                 const types::DroneConfigData& drone,
                                                 const Position3D& center) {
    const types::MapConfig map_config = hidden_map.getMapConfig();
    const BoundsCm bounds = boundsFromConfig(map_config);
    const double radius_cm = physicalCm(drone.radius);
    const double center_x = xCm(center.x);
    const double center_y = yCm(center.y);
    const double center_z = zCm(center.z);

    if (radius_cm < 0.0) {
        return PlacementCheck{PlacementProblem::DataError, "drone radius is negative."};
    }

    std::vector<std::string> out_of_bounds_reasons;
    if (center_x - radius_cm < bounds.min_x - kBoundaryEpsilonCm) {
        out_of_bounds_reasons.push_back("x_min");
    }
    if (center_x + radius_cm > bounds.max_x + kBoundaryEpsilonCm) {
        out_of_bounds_reasons.push_back("x_max");
    }
    if (center_y - radius_cm < bounds.min_y - kBoundaryEpsilonCm) {
        out_of_bounds_reasons.push_back("y_min");
    }
    if (center_y + radius_cm > bounds.max_y + kBoundaryEpsilonCm) {
        out_of_bounds_reasons.push_back("y_max");
    }
    if (center_z - radius_cm < bounds.min_z - kBoundaryEpsilonCm) {
        out_of_bounds_reasons.push_back("height_min");
    }
    if (center_z + radius_cm > bounds.max_z + kBoundaryEpsilonCm) {
        out_of_bounds_reasons.push_back("height_max");
    }

    if (!out_of_bounds_reasons.empty()) {
        std::ostringstream message;
        message << "center_cm=(" << center_x << ", " << center_y << ", " << center_z
                << "), radius_cm=" << radius_cm
                << ", map_bounds_cm=[x:" << bounds.min_x << ".." << bounds.max_x
                << ", y:" << bounds.min_y << ".." << bounds.max_y
                << ", height:" << bounds.min_z << ".." << bounds.max_z << "], failed_axes=";
        for (std::size_t index = 0; index < out_of_bounds_reasons.size(); ++index) {
            if (index != 0) {
                message << ",";
            }
            message << out_of_bounds_reasons[index];
        }
        return PlacementCheck{PlacementProblem::OutOfBounds, message.str()};
    }

    const double resolution_cm = physicalCm(map_config.resolution);
    if (resolution_cm <= 0.0) {
        return PlacementCheck{PlacementProblem::DataError, "hidden map resolution is not positive."};
    }

    // Tangency is allowed, so only count overlap that penetrates past the margin.
    const double collision_radius_cm = std::max(0.0, radius_cm - kBoundaryEpsilonCm);

    const auto cellCount = [resolution_cm](double min_value, double max_value) {
        return std::max(
            1, static_cast<int>(std::llround((max_value - min_value) / resolution_cm)));
    };
    const auto firstIndex = [resolution_cm](double value, double min_value) {
        return static_cast<int>(std::floor((value - min_value) / resolution_cm));
    };

    const int x_count = cellCount(bounds.min_x, bounds.max_x);
    const int y_count = cellCount(bounds.min_y, bounds.max_y);
    const int z_count = cellCount(bounds.min_z, bounds.max_z);

    const int min_x_index =
        std::clamp(firstIndex(center_x - radius_cm, bounds.min_x), 0, x_count - 1);
    const int max_x_index =
        std::clamp(firstIndex(center_x + radius_cm, bounds.min_x), 0, x_count - 1);
    const int min_y_index =
        std::clamp(firstIndex(center_y - radius_cm, bounds.min_y), 0, y_count - 1);
    const int max_y_index =
        std::clamp(firstIndex(center_y + radius_cm, bounds.min_y), 0, y_count - 1);
    const int min_z_index =
        std::clamp(firstIndex(center_z - radius_cm, bounds.min_z), 0, z_count - 1);
    const int max_z_index =
        std::clamp(firstIndex(center_z + radius_cm, bounds.min_z), 0, z_count - 1);

    try {
        for (int z_index = min_z_index; z_index <= max_z_index; ++z_index) {
            for (int y_index = min_y_index; y_index <= max_y_index; ++y_index) {
                for (int x_index = min_x_index; x_index <= max_x_index; ++x_index) {
                    const double voxel_min_x = bounds.min_x + x_index * resolution_cm;
                    const double voxel_max_x = voxel_min_x + resolution_cm;
                    const double voxel_min_y = bounds.min_y + y_index * resolution_cm;
                    const double voxel_max_y = voxel_min_y + resolution_cm;
                    const double voxel_min_z = bounds.min_z + z_index * resolution_cm;
                    const double voxel_max_z = voxel_min_z + resolution_cm;

                    if (!sphereIntersectsVoxel(center_x,
                                               center_y,
                                               center_z,
                                               collision_radius_cm,
                                               voxel_min_x,
                                               voxel_max_x,
                                               voxel_min_y,
                                               voxel_max_y,
                                               voxel_min_z,
                                               voxel_max_z)) {
                        continue;
                    }

                    const Position3D voxel_center{
                        (voxel_min_x + 0.5 * resolution_cm) * x_extent[cm],
                        (voxel_min_y + 0.5 * resolution_cm) * y_extent[cm],
                        (voxel_min_z + 0.5 * resolution_cm) * z_extent[cm],
                    };
                    if (hidden_map.atVoxel(voxel_center) == types::VoxelOccupancy::Occupied) {
                        std::ostringstream message;
                        message << "near voxel center_cm=(" << voxel_min_x + 0.5 * resolution_cm
                                << ", " << voxel_min_y + 0.5 * resolution_cm << ", "
                                << voxel_min_z + 0.5 * resolution_cm << ").";
                        return PlacementCheck{PlacementProblem::Collision, message.str()};
                    }
                }
            }
        }
    } catch (const std::exception& exception) {
        return PlacementCheck{PlacementProblem::DataError,
                              std::string("while reading hidden map: ") + exception.what()};
    }

    return PlacementCheck{};
}

void writePreflightErrorLog(const std::filesystem::path& output_map_file,
                            const types::ErrorRef& error) {
    const std::filesystem::path run_folder = output_map_file.parent_path();
    if (!run_folder.empty()) {
        std::filesystem::create_directories(run_folder);
    }

    const std::filesystem::path log_file = run_folder / "error.log";
    std::ofstream log{log_file};
    if (!log) {
        throw std::runtime_error("Failed to create preflight error log: " + log_file.string());
    }

    log << "Mission preflight failed before the first drone step.\n"
        << "code: " << error.code << '\n'
        << "message: " << error.message << '\n'
        << "steps: 0\n"
        << "output_map: not written; no mapping step, drone-body marking, or lidar scan ran.\n";
}

// Non-throwing: an in-flight failure must still let the run finish gracefully
// (save the partial map, report the error) even if the log cannot be written.
void writeInFlightErrorLog(const std::filesystem::path& output_map_file,
                           const types::ErrorRef& error,
                           std::size_t steps) {
    try {
        const std::filesystem::path run_folder = output_map_file.parent_path();
        if (!run_folder.empty()) {
            std::filesystem::create_directories(run_folder);
        }
        std::ofstream log{run_folder / "error.log"};
        if (log) {
            log << "Mission failed during flight.\n"
                << "code: " << error.code << '\n'
                << "message: " << error.message << '\n'
                << "steps: " << steps << '\n';
        }
    } catch (...) {
        // Logging must never abort the run.
    }
}

} // namespace

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

types::MissionRunResult MissionControlImpl::runMission() {
    auto saveOutputMap = [this](std::vector<types::ErrorRef>& errors) { // saves the output map to the file
        try {
            output_map_.save(output_map_file_);
            return true;
        } catch (const std::exception& exception) {
            addError(errors, "OUTPUT_MAP_SAVE_FAILED", exception.what());
        } catch (...) {
            addError(errors, "OUTPUT_MAP_SAVE_FAILED", "Unknown error while saving output map.");
        }
        return false;
    };

    const PlacementCheck preflight =
        checkDronePlacement(hidden_map_, drone_, drone_control_.state().position);
    if (preflight.problem != PlacementProblem::None) {
        const types::ErrorRef error = preflightError(preflight);
        std::vector<types::ErrorRef> errors{error};
        try {
            writePreflightErrorLog(output_map_file_, error);
        } catch (const std::exception& exception) {
            addError(errors, "PREFLIGHT_ERROR_LOG_WRITE_FAILED", exception.what());
        }

        return types::MissionRunResult{types::MissionRunStatus::Error, 0, std::move(errors)};
    }

    for (std::size_t executed_steps = 0; executed_steps < mission_.max_steps; ++executed_steps) {
        const types::DroneStepResult step_result = drone_control_.step(); // performs one logical drone step
        const std::size_t completed_steps = executed_steps + 1; // increments the completed steps

        // Simulator-side physical check: did this step push the drone body into
        // an occupied hidden voxel or outside the real map? Planning never sees
        // the hidden map, so this is where a physically illegal move is caught.
        if (step_result.status != types::DroneStepStatus::Error) {
            const PlacementCheck physical =
                checkDronePlacement(hidden_map_, drone_, drone_control_.state().position);
            if (physical.problem != PlacementProblem::None) {
                const types::ErrorRef error = inFlightError(physical);
                std::vector<types::ErrorRef> errors{error};
                writeInFlightErrorLog(output_map_file_, error, completed_steps);
                saveOutputMap(errors);
                return types::MissionRunResult{types::MissionRunStatus::Error,
                                               completed_steps,
                                               std::move(errors)};
            }
        }

        if (step_result.status == types::DroneStepStatus::Completed) { // if the drone step is completed save the output map
            std::vector<types::ErrorRef> errors;
            if (!saveOutputMap(errors)) { // if the output map is not saved, return an error
                return types::MissionRunResult{types::MissionRunStatus::Error,
                                               completed_steps,
                                               std::move(errors)};
            }

            return types::MissionRunResult{types::MissionRunStatus::Completed,
                                           completed_steps,
                                           {}};
        }

        if (step_result.status == types::DroneStepStatus::Error) { // if the drone step is an error, return an error
            std::vector<types::ErrorRef> errors;
            addError(errors,
                     "DRONE_STEP_FAILED",
                     step_result.message.empty() ? "Drone control step failed."
                                                 : step_result.message);
            saveOutputMap(errors); //
            return types::MissionRunResult{types::MissionRunStatus::Error,
                                           completed_steps,
                                           std::move(errors)};
        }
    }

    std::vector<types::ErrorRef> errors;
    if (!saveOutputMap(errors)) {
        return types::MissionRunResult{types::MissionRunStatus::Error,
                                       mission_.max_steps,
                                       std::move(errors)};
    }

    return types::MissionRunResult{types::MissionRunStatus::MaxSteps,
                                   mission_.max_steps,
                                   {}};
}

} // namespace drone_mapper
