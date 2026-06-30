#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

// **Do not change this interface.**
class ILidar {
public:
    virtual ~ILidar() = default;

    [[nodiscard]] virtual types::LidarScanResult scan(Orientation scan_orientation) const = 0; // return array of beams, each beam is a LidarHit object

    [[nodiscard]] virtual types::LidarConfigData config() const = 0; // return lidar capabilities

};

} // namespace drone_mapper
