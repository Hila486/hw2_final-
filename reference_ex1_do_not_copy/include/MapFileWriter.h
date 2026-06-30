#pragma once

#include <string>

#include "SparseBuildingMap.h"
#include "MissionConfig.h"

/*
    MapFileWriter

    Responsible for writing the drone's discovered map into map_output.txt.

    Important:
    This writes SparseBuildingMap, not GroundTruthMap.

    GroundTruthMap = hidden real world.
    SparseBuildingMap = what the drone discovered.

    The missionConfig is needed because cells outside the required
    mission boundaries should be written as -2 = OutOfBounds.
*/
class MapFileWriter {
public:
    static bool writeSparseMap(
        const std::string& filePath,
        const SparseBuildingMap& map,
        const MissionConfig& missionConfig
    );
};