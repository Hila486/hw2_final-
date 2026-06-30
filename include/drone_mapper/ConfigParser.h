#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>

namespace drone_mapper {

class ConfigParser {
public:
    [[nodiscard]] static types::SimulationCompositionData
    parseSimulationComposition(const std::filesystem::path& filepath);

    [[nodiscard]] static types::SimulationConfigData
    parseSimulationConfig(const std::filesystem::path& filepath);

    [[nodiscard]] static types::MissionConfigData
    parseMissionConfig(const std::filesystem::path& filepath);

    [[nodiscard]] static types::DroneConfigData
    parseDroneConfig(const std::filesystem::path& filepath);

    [[nodiscard]] static types::LidarConfigData
    parseLidarConfig(const std::filesystem::path& filepath);
};

} // namespace drone_mapper
