// SparseBuildingMap.cpp
#include "SparseBuildingMap.h"

// Constructor.
SparseBuildingMap::SparseBuildingMap(int sizeX, int sizeY, int sizeZ)
    : GridMap(sizeX, sizeY, sizeZ, CellState::Unknown)
{
}
// IBuildingMap interface implementation:
CellState SparseBuildingMap::getCell(const Position& pos) const {
    return GridMap::getCell(pos);
}

void SparseBuildingMap::setCell(const Position& pos, CellState state) {
    GridMap::setCell(pos, state);
}