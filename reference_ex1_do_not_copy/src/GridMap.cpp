#include "GridMap.h"

namespace {
    // Helper function to convert a length in centimeters to an integer index.
    int cmToInt(Cm value) {
        return static_cast<int>(value.numerical_value_in(cm));
    }
}

// Constructor.
GridMap::GridMap(int sizeX, int sizeY, int sizeZ, CellState initialState)
    : sizeX(sizeX),
      sizeY(sizeY),
      sizeZ(sizeZ),
      cells(sizeX * sizeY * sizeZ, initialState)
{
}

// Checks whether a given position is inside the map boundaries.
bool GridMap::isInside(const Position& pos) const {
    return pos.x >= 0 * cm &&
           pos.x < sizeX * cm &&
           pos.y >= 0 * cm &&
           pos.y < sizeY * cm &&
           pos.height >= 0 * cm &&
           pos.height < sizeZ * cm;
}

// Converts a 3D position into a 1D vector index. Order: height layer -> y row -> x column
int GridMap::index(const Position& pos) const {
    int x = cmToInt(pos.x);
    int y = cmToInt(pos.y);
    int z = cmToInt(pos.height);

    return z * sizeX * sizeY + y * sizeX + x;
}

// Returns the cell state at the given position. If the position is outside the map, returns OutOfBounds.
CellState GridMap::getCell(const Position& pos) const {
    if (!isInside(pos)) {
        return CellState::OutOfBounds;
    }

    return cells[index(pos)];
}

// Updates the cell state at the given position. If the position is outside the map, does nothing.
void GridMap::setCell(const Position& pos, CellState state) {
    if (!isInside(pos)) {
        return;
    }

    cells[index(pos)] = state;
}

// Getters for map size in each dimension:
int GridMap::getSizeX() const {
    return sizeX;
}

int GridMap::getSizeY() const {
    return sizeY;
}

int GridMap::getSizeZ() const {
    return sizeZ;
}