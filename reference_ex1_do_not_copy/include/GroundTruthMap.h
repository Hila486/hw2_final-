#pragma once
// GroundTruthMap.h
#include "GridMap.h"

/*
 GroundTruthMap represents the real hidden building/world map that the drone is trying to discover.
 It is loaded from map_input.txt and is not directly accessible to the drone.
*/
class GroundTruthMap : public GridMap {
public:
    // Constructor.
    GroundTruthMap(int sizeX, int sizeY, int sizeZ);
};