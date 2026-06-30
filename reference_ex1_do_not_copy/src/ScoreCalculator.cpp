#include "ScoreCalculator.h"

#include <algorithm>

namespace {
    bool isInsideMissionBounds(
        const Position& position,
        const MissionConfig& missionConfig
    ) {
        return
            position.x >= missionConfig.minX &&
            position.x <= missionConfig.maxX &&
            position.y >= missionConfig.minY &&
            position.y <= missionConfig.maxY &&
            position.height >= missionConfig.minZ &&
            position.height <= missionConfig.maxZ;
    }
}

/*
    calculateScore

    Compares the real hidden world map with the drone's discovered map.

    groundTruthMap:
        The real map loaded from map_input.txt.
        This is the "answer".

    droneMap:
        The map discovered by the drone.
        This is what we wrote to map_output.txt.

    missionConfig:
        Defines which cells are required to be mapped.

    Simple score formula:

        score = correct cells / total cells * 100

    Rules:
        - If a cell is outside mission boundaries:
              expected state = OutOfBounds (-2)

        - If a cell is inside mission boundaries:
              expected state = real state from GroundTruthMap

        - The drone state is checked the same way:
              outside mission boundaries should be OutOfBounds (-2)
              inside mission boundaries comes from SparseBuildingMap

    Examples:
        real map:  Occupied
        drone map: Occupied
        inside mission bounds
        => correct

        real map:  Free
        drone map: Unknown
        inside mission bounds
        => wrong

        outside mission bounds:
        expected: OutOfBounds
        drone:    OutOfBounds
        => correct

    The function returns a number between 0 and 100.
*/
double ScoreCalculator::calculateScore(
    const GroundTruthMap& groundTruthMap,
    const SparseBuildingMap& droneMap,
    const MissionConfig& missionConfig
) {
    /*
        We use the smaller size in each dimension.

        Usually both maps should have the same size.
        But using std::min protects us if something is different by mistake.
    */
    int sizeX = std::min(groundTruthMap.getSizeX(), droneMap.getSizeX());
    int sizeY = std::min(groundTruthMap.getSizeY(), droneMap.getSizeY());
    int sizeZ = std::min(groundTruthMap.getSizeZ(), droneMap.getSizeZ());

    int totalCells = sizeX * sizeY * sizeZ;

    if (totalCells == 0) {
        return 0.0;
    }

    int correctCells = 0;

    /*
        Go over every cell in the map.

        Loop order:
        1. height layer
        2. y row
        3. x column
    */
    for (int height = 0; height < sizeZ; ++height) {
        for (int y = 0; y < sizeY; ++y) {
            for (int x = 0; x < sizeX; ++x) {
                Position pos{x * cm, y * cm, height * cm};

                CellState expectedState;
                CellState actualState;

                if (!isInsideMissionBounds(pos, missionConfig)) {
                    expectedState = CellState::OutOfBounds;
                    actualState = CellState::OutOfBounds;
                } else {
                    expectedState = groundTruthMap.getCell(pos);
                    actualState = droneMap.getCell(pos);
                }

                if (expectedState == actualState) {
                    ++correctCells;
                }
            }
        }
    }

    return (static_cast<double>(correctCells) / totalCells) * 100.0;
}