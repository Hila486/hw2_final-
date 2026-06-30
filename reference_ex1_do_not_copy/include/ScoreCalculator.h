#pragma once

#include "GroundTruthMap.h"
#include "SparseBuildingMap.h"
#include "MissionConfig.h"

/*
    ScoreCalculator

    This class compares two maps:

    1. GroundTruthMap
       The real hidden world that came from map_input.txt.

    2. SparseBuildingMap
       The drone's discovered map that we write to map_output.txt.

    3. MissionConfig
       Defines the required mapping boundaries.

    For HW1, we are allowed to choose our own score formula.
    Our simple formula is:

        score = correct cells / total cells * 100

    Important:
    Cells outside mission boundaries are expected to be OutOfBounds (-2).
*/
class ScoreCalculator {
public:
    static double calculateScore(
        const GroundTruthMap& groundTruthMap,
        const SparseBuildingMap& droneMap,
        const MissionConfig& missionConfig
    );
};