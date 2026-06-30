#pragma once
// DroneConfig.h

#include "Units.h"

// DroneConfig represents the drone's physical and sensor capabilities, as defined in drone_config.txt.

struct DroneConfig {
    // Minimum passage dimensions
    Cm minPassageWidth = 0 * cm;
    Cm minPassageLength = 0 * cm;
    Cm minPassageHeight = 0 * cm;

    // Maximum movement capabilities
    Cm maxAdvance = 0 * cm;
    Cm maxElevate = 0 * cm;
    Degree maxRotate = 0 * deg;

    // Lidar specifications
    Cm lidarMinRange = 0 * cm;
    Cm lidarMaxRange = 0 * cm;
    Cm lidarBeamSpacing = 0 * cm;

    // Number of circles of lidar beams
    int lidarFovCircleCount = 1;
};