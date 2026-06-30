#pragma once
// MissionConfig.h
#include "Types.h"

// Defines the MissionConfig struct, which stores mission settings such as
// start position, start angle, map boundaries, and resolution.
struct MissionConfig {
    
    // Starting drone pose
    Position startPosition{0 * cm, 0 * cm, 0 * cm};
    Degree startAngleDeg = 0 * deg;

    // Mapping boundaries
    Cm minX = 0 * cm;
    Cm maxX = 0 * cm;

    Cm minY = 0 * cm;
    Cm maxY = 0 * cm;

    Cm minZ = 0 * cm;
    Cm maxZ = 0 * cm;

    // Required output/map resolution
    Cm resolutionCm = SUPPORTED_RESOLUTION_CM * cm;
};