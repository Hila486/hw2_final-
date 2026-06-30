#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/ScanResultToVoxels.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <utility>

namespace drone_mapper {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumPositiveStepCm = 1.0e-6;
constexpr double kFallbackSafetyStepCm = 1.0;

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

[[nodiscard]] double horizontalDegrees(HorizontalAngle angle) {
    return angle.force_numerical_value_in(deg);
}

[[nodiscard]] double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

[[nodiscard]] double squaredDistanceCm(const Position3D& a, const Position3D& b) {
    const double dx_cm = xCm(a.x) - xCm(b.x);
    const double dy_cm = yCm(a.y) - yCm(b.y);
    const double dz_cm = zCm(a.z) - zCm(b.z);
    return dx_cm * dx_cm + dy_cm * dy_cm + dz_cm * dz_cm;
}

[[nodiscard]] bool isInsideSphere(const Position3D& position,
                                  const Position3D& center,
                                  double radius_cm) {
    return squaredDistanceCm(position, center) <= radius_cm * radius_cm;
}

[[nodiscard]] types::DroneStepResult continueResult(std::string message = {}) {
    return types::DroneStepResult{types::DroneStepStatus::Continue, std::move(message)};
}

[[nodiscard]] types::DroneStepResult completedResult(std::string message = {}) {
    return types::DroneStepResult{types::DroneStepStatus::Completed, std::move(message)};
}

[[nodiscard]] types::DroneStepResult errorResult(std::string message) {
    return types::DroneStepResult{types::DroneStepStatus::Error, std::move(message)};
}

} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    try {
        markCurrentDroneBodyAsEmpty();

        const types::LidarScanResult* latest_scan =
            latest_scan_ ? &(*latest_scan_) : nullptr;
        const types::MappingStepCommand command =
            mapping_algorithm_.nextStep(state(), latest_scan);

        // The latest scan is offered to exactly one algorithm call.
        latest_scan_.reset();

        if (command.status == types::AlgorithmStatus::Finished) {
            return completedResult("Mapping algorithm finished.");
        }
        if (command.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            return completedResult("Mapping algorithm finished with unmappable voxels.");
        }

        if (command.movement) {
            std::string movement_message;
            if (!executeMovementCommand(*command.movement, movement_message)) {
                ++step_index_;
                return continueResult(std::move(movement_message));
            }
        }

        if (command.scan_orientation) {
            latest_scan_ = lidar_.scan(*command.scan_orientation);
            ScanResultToVoxels::applyToMap(
                output_map_,
                gps_.position(),
                gps_.heading(),
                *latest_scan_,
                lidar_.config());
        }

        ++step_index_;
        return continueResult();
    } catch (const std::exception& exception) {
        return errorResult(std::string("DroneControlImpl::step failed: ") + exception.what());
    }
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

bool DroneControlImpl::executeMovementCommand(const types::MovementCommand& command,
                                              std::string& message) {
    if (command.type == types::MovementCommandType::Hover) {
        return true;
    }

    if (!isMovementCommandWithinLimits(command, message)) {
        return false;
    }

    const Position3D start = gps_.position();
    const Orientation heading = gps_.heading();
    const Position3D target = targetPositionForMovement(start, heading, command);

    if (command.type != types::MovementCommandType::Rotate &&
        !isSweptSphereKnownEmpty(start, target)) {
        message = "Movement rejected: swept drone volume is not fully known empty.";
        return false;
    }

    types::MovementResult result;
    switch (command.type) {
        case types::MovementCommandType::Rotate:
            result = movement_.rotate(command.rotation, command.angle);
            break;
        case types::MovementCommandType::Advance:
            result = movement_.advance(command.distance);
            break;
        case types::MovementCommandType::Elevate:
            result = movement_.elevate(command.distance);
            break;
        case types::MovementCommandType::Hover:
            return true;
    }

    if (!result) {
        message = result.message.empty() ? "Movement command failed." : result.message;
        return false;
    }

    return true;
}

bool DroneControlImpl::isMovementCommandWithinLimits(
    const types::MovementCommand& command,
    std::string& message) const {
    switch (command.type) {
        case types::MovementCommandType::Hover:
            return true;
        case types::MovementCommandType::Rotate: {
            const double angle_degrees = std::abs(horizontalDegrees(command.angle));
            const double max_degrees = horizontalDegrees(drone_.max_rotate);
            if (angle_degrees > max_degrees) {
                message = "Movement rejected: rotate command exceeds max_rotate.";
                return false;
            }
            return true;
        }
        case types::MovementCommandType::Advance: {
            const double distance_cm = physicalCm(command.distance);
            if (distance_cm < 0.0) {
                message = "Movement rejected: advance distance is negative.";
                return false;
            }
            if (distance_cm > physicalCm(drone_.max_advance)) {
                message = "Movement rejected: advance command exceeds max_advance.";
                return false;
            }
            return true;
        }
        case types::MovementCommandType::Elevate: {
            if (std::abs(physicalCm(command.distance)) > physicalCm(drone_.max_elevate)) {
                message = "Movement rejected: elevate command exceeds max_elevate.";
                return false;
            }
            return true;
        }
    }

    message = "Movement rejected: unknown movement command type.";
    return false;
}

Position3D DroneControlImpl::targetPositionForMovement(
    const Position3D& start,
    const Orientation& heading,
    const types::MovementCommand& command) const {
    if (command.type == types::MovementCommandType::Advance) {
        const double distance_cm = physicalCm(command.distance);
        const double radians = degreesToRadians(horizontalDegrees(heading.horizontal));
        return Position3D{
            start.x + distance_cm * std::cos(radians) * x_extent[cm],
            start.y + distance_cm * std::sin(radians) * y_extent[cm],
            start.z,
        };
    }

    if (command.type == types::MovementCommandType::Elevate) {
        return Position3D{
            start.x,
            start.y,
            start.z + physicalCm(command.distance) * z_extent[cm],
        };
    }

    return start;
}

bool DroneControlImpl::isCenterInsideMapBounds(const Position3D& center) const {
    return output_map_.isInBounds(center);
}

bool DroneControlImpl::isSphereKnownEmptyAt(
    const Position3D& center,
    const Position3D& already_free_center) const {
    const double radius_cm = std::max(0.0, physicalCm(drone_.radius));
    const double sample_step_cm = safetySampleStepCm();

    if (!isCenterInsideMapBounds(center)) {
        return false;
    }
    if (!isInsideSphere(center, already_free_center, radius_cm) &&
        output_map_.atVoxel(center) != types::VoxelOccupancy::Empty) {
        return false;
    }

    /*
     * The current drone body is trusted as already free because GPS says the
     * drone is there now. We still block out-of-bounds samples, but stale
     * Occupied/PotentiallyOccupied/Unmapped values inside that original body do
     * not reject movement.
     */
    for (double dz_cm = -radius_cm; dz_cm <= radius_cm; dz_cm += sample_step_cm) {
        for (double dy_cm = -radius_cm; dy_cm <= radius_cm; dy_cm += sample_step_cm) {
            for (double dx_cm = -radius_cm; dx_cm <= radius_cm; dx_cm += sample_step_cm) {
                const Position3D checked_position{
                    center.x + dx_cm * x_extent[cm],
                    center.y + dy_cm * y_extent[cm],
                    center.z + dz_cm * z_extent[cm],
                };

                if (!isInsideSphere(checked_position, center, radius_cm)) {
                    continue;
                }

                if (!isCenterInsideMapBounds(checked_position)) {
                    return false;
                }
                if (isInsideSphere(checked_position, already_free_center, radius_cm)) {
                    continue;
                }
                if (output_map_.atVoxel(checked_position) != types::VoxelOccupancy::Empty) {
                    return false;
                }
            }
        }
    }

    return true;
}

bool DroneControlImpl::isSweptSphereKnownEmpty(const Position3D& start,
                                               const Position3D& end) const {
    const double dx_cm = xCm(end.x) - xCm(start.x);
    const double dy_cm = yCm(end.y) - yCm(start.y);
    const double dz_cm = zCm(end.z) - zCm(start.z);
    const double distance_cm = std::sqrt(dx_cm * dx_cm + dy_cm * dy_cm + dz_cm * dz_cm);
    const double sample_step_cm = safetySampleStepCm();
    const std::size_t sample_count =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(distance_cm / sample_step_cm)));

    for (std::size_t sample_index = 0; sample_index <= sample_count; ++sample_index) {
        const double t = static_cast<double>(sample_index) / static_cast<double>(sample_count);
        const Position3D center{
            start.x + (dx_cm * t) * x_extent[cm],
            start.y + (dy_cm * t) * y_extent[cm],
            start.z + (dz_cm * t) * z_extent[cm],
        };

        if (!isSphereKnownEmptyAt(center, start)) {
            return false;
        }
    }

    return true;
}

double DroneControlImpl::safetySampleStepCm() const {
    double resolution_cm = physicalCm(output_map_.getMapConfig().resolution);
    if (resolution_cm <= 0.0) {
        resolution_cm = physicalCm(mission_.gps_resolution);
    }
    if (resolution_cm <= 0.0) {
        resolution_cm = 2.0 * kFallbackSafetyStepCm;
    }

    return std::max(kMinimumPositiveStepCm, resolution_cm * 0.5);
}

void DroneControlImpl::markCurrentDroneBodyAsEmpty() {
    const Position3D center = gps_.position();
    const double radius_cm = std::max(0.0, physicalCm(drone_.radius));
    const double sample_step_cm = safetySampleStepCm();

    const auto markEmptyIfNotOccupied = [this](const Position3D& position) {
        if (!isCenterInsideMapBounds(position)) {
            return;
        }
        // The drone body is genuinely free, but we never demote a voxel that
        // lidar has already classified Occupied: a voxel only partially covered
        // by the body may still contain a real obstacle, so trust the sensor.
        if (output_map_.atVoxel(position) == types::VoxelOccupancy::Occupied) {
            return;
        }
        output_map_.set(position, types::VoxelOccupancy::Empty);
    };

    markEmptyIfNotOccupied(center);

    for (double dz_cm = -radius_cm; dz_cm <= radius_cm; dz_cm += sample_step_cm) {
        for (double dy_cm = -radius_cm; dy_cm <= radius_cm; dy_cm += sample_step_cm) {
            for (double dx_cm = -radius_cm; dx_cm <= radius_cm; dx_cm += sample_step_cm) {
                const Position3D checked_position{
                    center.x + dx_cm * x_extent[cm],
                    center.y + dy_cm * y_extent[cm],
                    center.z + dz_cm * z_extent[cm],
                };

                if (isInsideSphere(checked_position, center, radius_cm)) {
                    markEmptyIfNotOccupied(checked_position);
                }
            }
        }
    }
}

} // namespace drone_mapper
