#pragma once

#include "GridMap.h"
#include "IBuildingMap.h"

// SparseBuildingMap is the map that the drone builds during its exploration.
// It starts with all cells as Unknown, and the drone updates it with Free and Occupied states as it discovers the environment.
class SparseBuildingMap : public IBuildingMap, public GridMap {
public:
    // Constructor.
    SparseBuildingMap(int sizeX, int sizeY, int sizeZ);

    // IBuildingMap interface implementation:
    CellState getCell(const Position& pos) const override;
    void setCell(const Position& pos, CellState state) override;
};