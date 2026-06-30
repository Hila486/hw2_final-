#pragma once

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMutableMap3D.h>

#include <optional>
#include <string>

namespace drone_mapper {

class DroneControlImpl final : public IDroneControl {
public:
    DroneControlImpl(types::DroneConfigData drone,
                     types::MissionConfigData mission,
                     ILidar& lidar,
                     IGPS& gps,
                     IDroneMovement& movement,
                     IMutableMap3D& output_map,
                     IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] types::DroneStepResult step() override;
    [[nodiscard]] types::DroneState state() const override;

private:
    [[nodiscard]] bool executeMovementCommand(const types::MovementCommand& command,
                                              std::string& message);
    [[nodiscard]] bool isMovementCommandWithinLimits(const types::MovementCommand& command,
                                                     std::string& message) const;
    [[nodiscard]] Position3D targetPositionForMovement(
        const Position3D& start,
        const Orientation& heading,
        const types::MovementCommand& command) const;
    [[nodiscard]] bool isCenterInsideMapBounds(const Position3D& center) const;
    [[nodiscard]] bool isSphereKnownEmptyAt(const Position3D& center,
                                            const Position3D& already_free_center) const;
    [[nodiscard]] bool isSweptSphereKnownEmpty(const Position3D& start,
                                               const Position3D& end) const;
    [[nodiscard]] double safetySampleStepCm() const;
    void markCurrentDroneBodyAsEmpty();

    types::DroneConfigData drone_;
    types::MissionConfigData mission_;
    ILidar& lidar_;
    IGPS& gps_;
    IDroneMovement& movement_;
    IMutableMap3D& output_map_;
    IMappingAlgorithm& mapping_algorithm_;
    std::optional<types::LidarScanResult> latest_scan_;
    std::size_t step_index_ = 0;

};

} // namespace drone_mapper
