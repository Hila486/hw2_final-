#pragma once
// IBuildingMap.h

#include "Types.h"

// Defines the IBuildingMap interface, which represents a 3D map of the building that the drone is exploring.
// The drone interacts with the map through this interface, without knowing the concrete map
class IBuildingMap {
public:
    // Virtual destructor for proper cleanup of derived classes.
    virtual ~IBuildingMap() = default;

    // Returns the state of the cell at the given position.
    virtual CellState getCell(const Position& pos) const = 0;
    
    // Updates the state of the cell at the given position.
    virtual void setCell(const Position& pos, CellState state) = 0;
};


