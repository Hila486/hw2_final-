#include <drone_mapper/MappingAlgorithmImpl.h>

#include <drone_mapper/IMap3D.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace drone_mapper {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilonDegrees = 1.0e-6;
constexpr double kPositionEpsilonCm = 1.0e-6;
constexpr double kMinimumStepCm = 1.0;
constexpr std::size_t kScanDirectionCount = 6;
constexpr std::size_t kMaxFailedTranslations = 24;

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

[[nodiscard]] double normalizeDegrees(double degrees) {
    double normalized = std::fmod(degrees, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized;
}

[[nodiscard]] double signedAngleDeltaDegrees(double from_degrees, double to_degrees) {
    double delta = normalizeDegrees(to_degrees) - normalizeDegrees(from_degrees);
    if (delta > 180.0) {
        delta -= 360.0;
    }
    if (delta < -180.0) {
        delta += 360.0;
    }
    return delta;
}

[[nodiscard]] double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

[[nodiscard]] bool isPositive(PhysicalLength length) {
    return physicalCm(length) > 0.0;
}

[[nodiscard]] bool isPositiveAngle(HorizontalAngle angle) {
    return angle > 0.0 * horizontal_angle[deg];
}

[[nodiscard]] Position3D offsetHorizontal(
    const Position3D& position,
    double heading_degrees,
    PhysicalLength distance) {
    const double radians = degreesToRadians(heading_degrees);
    const double distance_cm = physicalCm(distance);

    return Position3D{
        position.x + distance_cm * std::cos(radians) * x_extent[cm],
        position.y + distance_cm * std::sin(radians) * y_extent[cm],
        position.z,
    };
}

[[nodiscard]] Position3D offsetVertical(
    const Position3D& position,
    PhysicalLength distance) {
    const double distance_cm = physicalCm(distance);

    return Position3D{
        position.x,
        position.y,
        position.z + distance_cm * z_extent[cm],
    };
}

[[nodiscard]] types::MappingStepCommand movementCommand(types::MovementCommand movement) {
    types::MappingStepCommand command;
    command.movement = movement;
    command.status = types::AlgorithmStatus::Working;
    return command;
}

[[nodiscard]] bool isBlockedOccupancy(types::VoxelOccupancy occupancy) {
    return occupancy == types::VoxelOccupancy::Occupied ||
           occupancy == types::VoxelOccupancy::OutOfBounds ||
           occupancy == types::VoxelOccupancy::PotentiallyOccupied;
}

} // namespace

bool MappingAlgorithmImpl::GridCell::operator<(const GridCell& other) const {
    if (x != other.x) {
        return x < other.x;
    }
    if (y != other.y) {
        return y < other.y;
    }
    return z < other.z;
}

bool MappingAlgorithmImpl::GridCell::operator==(const GridCell& other) const {
    return x == other.x && y == other.y && z == other.z;
}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                       const types::LidarScanResult* latest_scan) {
    const GridCell current_cell = cellFromPosition(state.position);

    handlePendingTranslationResult(current_cell);
    initializeAtCurrentCell(current_cell, state.position);

    if (failed_translation_count_ >= kMaxFailedTranslations) {
        return finishCommand();
    }

    if (scan_cell_ != current_cell) {
        resetScanSequenceForCell(current_cell);
    }

    if (waiting_for_scan_result_) {
        if (latest_scan == nullptr) {
            return scanCommand(*pending_scan_index_);
        }

        waiting_for_scan_result_ = false;
        ++next_scan_index_;
        pending_scan_index_.reset();
    }

    if (next_scan_index_ < kScanDirectionCount) {
        return scanCommand(next_scan_index_);
    }

    if (!pending_target_) {
        pending_target_ = findExplorationTarget(state, current_cell);
    }

    if (pending_target_) {
        return commandForTarget(state, current_cell, *pending_target_);
    }

    if (dfs_path_.size() <= 1) {
        return finishCommand();
    }

    const PathNode previous_node = dfs_path_[dfs_path_.size() - 2];
    const double delta_x_cm = xCm(previous_node.position.x) - xCm(state.position.x);
    const double delta_y_cm = yCm(previous_node.position.y) - yCm(state.position.y);
    const double delta_z_cm = zCm(previous_node.position.z) - zCm(state.position.z);
    const bool backtrack_is_vertical =
        std::abs(delta_x_cm) <= kPositionEpsilonCm &&
        std::abs(delta_y_cm) <= kPositionEpsilonCm &&
        std::abs(delta_z_cm) > kPositionEpsilonCm;
    const double backtrack_distance_cm =
        backtrack_is_vertical ? std::abs(delta_z_cm) : std::hypot(delta_x_cm, delta_y_cm);

    pending_target_ = NavigationTarget{
        previous_node,
        true,
        backtrack_is_vertical,
        horizontalDegrees(state.heading.horizontal),
        backtrack_distance_cm * cm,
    };

    return commandForTarget(state, current_cell, *pending_target_);
}

MappingAlgorithmImpl::GridCell MappingAlgorithmImpl::cellFromPosition(
    const Position3D& position) const {
    const types::MapConfig map_config = output_map_.getMapConfig();
    const double step_cm = std::max(kMinimumStepCm, physicalCm(planningStep()));

    return GridCell{
        static_cast<int>(
            std::llround((xCm(position.x) - xCm(map_config.boundaries.min_x)) / step_cm)),
        static_cast<int>(
            std::llround((yCm(position.y) - yCm(map_config.boundaries.min_y)) / step_cm)),
        static_cast<int>(
            std::llround((zCm(position.z) - zCm(map_config.boundaries.min_height)) / step_cm)),
    };
}

PhysicalLength MappingAlgorithmImpl::planningStep() const {
    const types::MapConfig map_config = output_map_.getMapConfig();
    double step_cm = physicalCm(map_config.resolution);

    if (step_cm <= 0.0) {
        step_cm = physicalCm(mission_config_.gps_resolution);
    }
    if (step_cm <= 0.0) {
        step_cm = kMinimumStepCm;
    }

    if (isPositive(drone_config_.max_advance)) {
        step_cm = std::min(step_cm, physicalCm(drone_config_.max_advance));
    }
    if (isPositive(drone_config_.max_elevate)) {
        step_cm = std::min(step_cm, physicalCm(drone_config_.max_elevate));
    }

    return std::max(kMinimumStepCm, step_cm) * cm;
}

bool MappingAlgorithmImpl::isLegalDroneCenter(const Position3D& position) const {
    /*
     * The planner intentionally uses the map config bounds as physical/world
     * bounds, not mission_config_.mission_bounds. Mission bounds may describe
     * the scoring/mapping target area, while the drone may legally fly through
     * other in-world cells to reach it.
     */
    if (!output_map_.isInBounds(position)) {
        return false;
    }

    if (isBlockedOccupancy(output_map_.atVoxel(position))) {
        return false;
    }

    const double radius_cm = physicalCm(drone_config_.radius);
    if (radius_cm <= 0.0) {
        return true;
    }

    const double sample_step_cm =
        std::max(kMinimumStepCm, std::min(radius_cm, physicalCm(planningStep())));

    for (double dz_cm = -radius_cm; dz_cm <= radius_cm; dz_cm += sample_step_cm) {
        for (double dy_cm = -radius_cm; dy_cm <= radius_cm; dy_cm += sample_step_cm) {
            for (double dx_cm = -radius_cm; dx_cm <= radius_cm; dx_cm += sample_step_cm) {
                const double distance_squared =
                    dx_cm * dx_cm + dy_cm * dy_cm + dz_cm * dz_cm;
                if (distance_squared > radius_cm * radius_cm) {
                    continue;
                }

                const Position3D checked_position{
                    position.x + dx_cm * x_extent[cm],
                    position.y + dy_cm * y_extent[cm],
                    position.z + dz_cm * z_extent[cm],
                };

                if (!output_map_.isInBounds(checked_position) ||
                    isBlockedOccupancy(output_map_.atVoxel(checked_position))) {
                    return false;
                }
            }
        }
    }

    return true;
}

std::optional<MappingAlgorithmImpl::NavigationTarget>
MappingAlgorithmImpl::findExplorationTarget(
    const types::DroneState& state,
    const GridCell& current_cell) const {
    struct CandidateDirection {
        double relative_heading_degrees = 0.0;
        bool vertical = false;
        int vertical_sign = 0;
    };

    const PhysicalLength step = planningStep();
    const double current_heading_degrees = horizontalDegrees(state.heading.horizontal);

    const std::array<CandidateDirection, 6> directions{{
        CandidateDirection{0.0, false, 0},
        CandidateDirection{-90.0, false, 0},
        CandidateDirection{180.0, false, 0},
        CandidateDirection{90.0, false, 0},
        CandidateDirection{0.0, true, 1},
        CandidateDirection{0.0, true, -1},
    }};

    for (const CandidateDirection& direction : directions) {
        const double desired_heading_degrees =
            normalizeDegrees(current_heading_degrees + direction.relative_heading_degrees);
        const Position3D candidate_position =
            direction.vertical
                ? offsetVertical(state.position, direction.vertical_sign * step)
                : offsetHorizontal(state.position, desired_heading_degrees, step);
        const GridCell candidate_cell = cellFromPosition(candidate_position);

        if (candidate_cell == current_cell ||
            visited_cells_.find(candidate_cell) != visited_cells_.end() ||
            attempted_cells_.find(candidate_cell) != attempted_cells_.end() ||
            !isLegalDroneCenter(candidate_position)) {
            continue;
        }

        return NavigationTarget{
            PathNode{candidate_cell, candidate_position},
            false,
            direction.vertical,
            desired_heading_degrees,
            step,
        };
    }

    return std::nullopt;
}

types::MappingStepCommand MappingAlgorithmImpl::commandForTarget(
    const types::DroneState& state,
    const GridCell& current_cell,
    const NavigationTarget& target) {
    if (target.vertical) {
        const double delta_z_cm = zCm(target.node.position.z) - zCm(state.position.z);
        types::MovementCommand movement;
        movement.type = types::MovementCommandType::Elevate;
        movement.distance = delta_z_cm * cm;

        pending_translation_ = target;
        return movementCommand(movement);
    }

    const double dx_cm = xCm(target.node.position.x) - xCm(state.position.x);
    const double dy_cm = yCm(target.node.position.y) - yCm(state.position.y);
    const double horizontal_distance_cm = std::hypot(dx_cm, dy_cm);
    const double desired_heading_degrees =
        horizontal_distance_cm > 0.0
            ? normalizeDegrees(std::atan2(dy_cm, dx_cm) * 180.0 / kPi)
            : target.desired_heading_degrees;
    const double current_heading_degrees = horizontalDegrees(state.heading.horizontal);
    const double turn_degrees =
        signedAngleDeltaDegrees(current_heading_degrees, desired_heading_degrees);

    if (std::abs(turn_degrees) > kEpsilonDegrees) {
        double rotation_degrees = std::abs(turn_degrees);
        if (isPositiveAngle(drone_config_.max_rotate)) {
            rotation_degrees =
                std::min(rotation_degrees, horizontalDegrees(drone_config_.max_rotate));
        }

        types::MovementCommand movement;
        movement.type = types::MovementCommandType::Rotate;
        movement.rotation = turn_degrees >= 0.0
                                ? types::RotationDirection::Left
                                : types::RotationDirection::Right;
        movement.angle = rotation_degrees * horizontal_angle[deg];
        return movementCommand(movement);
    }

    types::MovementCommand movement;
    movement.type = types::MovementCommandType::Advance;
    movement.distance = horizontal_distance_cm * cm;

    pending_translation_ = NavigationTarget{
        target.node,
        target.backtracking,
        target.vertical,
        desired_heading_degrees,
        horizontal_distance_cm * cm,
    };

    (void)current_cell;
    return movementCommand(movement);
}

types::MappingStepCommand MappingAlgorithmImpl::scanCommand(std::size_t scan_index) {
    const std::array<Orientation, kScanDirectionCount> scans{{
        Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        Orientation{-90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        Orientation{180.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        Orientation{0.0 * horizontal_angle[deg], 90.0 * altitude_angle[deg]},
        Orientation{0.0 * horizontal_angle[deg], -90.0 * altitude_angle[deg]},
    }};

    types::MappingStepCommand command;
    command.scan_orientation = scans[scan_index];
    command.status = types::AlgorithmStatus::Working;

    waiting_for_scan_result_ = true;
    pending_scan_index_ = scan_index;
    return command;
}

types::MappingStepCommand MappingAlgorithmImpl::finishCommand() const {
    types::MappingStepCommand command;
    command.status = types::AlgorithmStatus::Finished;
    return command;
}

void MappingAlgorithmImpl::initializeAtCurrentCell(
    const GridCell& current_cell,
    const Position3D& position) {
    visited_cells_.insert(current_cell);

    if (dfs_path_.empty()) {
        dfs_path_.push_back(PathNode{current_cell, position});
    }
}

void MappingAlgorithmImpl::handlePendingTranslationResult(const GridCell& current_cell) {
    if (!pending_translation_) {
        return;
    }

    const NavigationTarget completed_target = *pending_translation_;
    pending_translation_.reset();

    if (current_cell == completed_target.node.cell) {
        failed_translation_count_ = 0;
        visited_cells_.insert(current_cell);

        if (completed_target.backtracking) {
            while (dfs_path_.size() > 1 && !(dfs_path_.back().cell == current_cell)) {
                dfs_path_.pop_back();
            }
        } else if (dfs_path_.empty() || !(dfs_path_.back().cell == current_cell)) {
            dfs_path_.push_back(completed_target.node);
        }

        pending_target_.reset();
        return;
    }

    ++failed_translation_count_;
    attempted_cells_.insert(completed_target.node.cell);
    pending_target_.reset();
}

void MappingAlgorithmImpl::resetScanSequenceForCell(const GridCell& current_cell) {
    scan_cell_ = current_cell;
    next_scan_index_ = 0;
    waiting_for_scan_result_ = false;
    pending_scan_index_.reset();
}

} // namespace drone_mapper
