#pragma once
// GridMap.h
#include <vector>

#include "Types.h"


 //Shared implementation for a 3D grid map used by both GroundTruthMap and SparseBuildingMap.
class GridMap {
public:
    // Constructor.
    GridMap(int sizeX, int sizeY, int sizeZ, CellState initialState);

    // Returns the cell state at the given position. If the position is outside the map, returns OutOfBounds.
    CellState getCell(const Position& pos) const;
    // Updates the cell state at the given position. If the position is outside the map, does nothing.
    void setCell(const Position& pos, CellState state);

    // Checks whether a given position is inside the map boundaries.
    bool isInside(const Position& pos) const;

    // Getters for map size in each dimension.
    int getSizeX() const;
    int getSizeY() const;
    int getSizeZ() const;

private:
    // Map dimensions
    int sizeX;
    int sizeY;
    int sizeZ;

    // We store the cell states in a 1D vector, and convert 3D positions to 1D indices when accessing it.
    std::vector<CellState> cells;

    // Converts a 3D position into a 1D vector index. Order: height layer -> y row -> x column
    int index(const Position& pos) const;
};