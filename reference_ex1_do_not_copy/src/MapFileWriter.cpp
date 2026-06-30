#include "MapFileWriter.h"

#include <fstream>
#include <iostream>

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
    Writes the map in the same simple format as map_input.txt:

    sizeX sizeY sizeZ
    then all cells layer by layer

    Cell values:
    0  = free / empty
    1  = occupied
    -1 = unknown / unmapped
    -2 = outside required mission boundaries

    Important:
    SparseBuildingMap stores what the drone discovered.
    MissionConfig defines which cells are required to be mapped.

    Therefore:
    - If a cell is outside mission boundaries, we write -2.
    - Otherwise, we write the value from SparseBuildingMap.

    Loop order:
    height layer -> y row -> x column
*/
bool MapFileWriter::writeSparseMap(
    const std::string& filePath,
    const SparseBuildingMap& map,
    const MissionConfig& missionConfig
) {
    std::ofstream file(filePath);

    if (!file.is_open()) {
        std::cout << "Could not create output map file: "
                  << filePath
                  << std::endl;

        return false;
    }

    int sizeX = map.getSizeX();
    int sizeY = map.getSizeY();
    int sizeZ = map.getSizeZ();

    file << sizeX << " " << sizeY << " " << sizeZ << "\n";

    for (int height = 0; height < sizeZ; ++height) {
        for (int y = 0; y < sizeY; ++y) {
            for (int x = 0; x < sizeX; ++x) {
                Position pos{x * cm, y * cm, height * cm};

                CellState state;

                if (!isInsideMissionBounds(pos, missionConfig)) {
                    state = CellState::OutOfBounds;
                } else {
                    state = map.getCell(pos);
                }

                file << static_cast<int>(state);

                if (x + 1 < sizeX) {
                    file << " ";
                }
            }

            file << "\n";
        }
    }

    return true;
}