// GroundTruthMap.cpp
#include "GroundTruthMap.h"

// Constructor.
GroundTruthMap::GroundTruthMap(int sizeX, int sizeY, int sizeZ)
    : GridMap(sizeX, sizeY, sizeZ, CellState::Free)
{
}