#pragma once
// DroneState.h
#include "Types.h"

// DroneState stores the drone's actual current state in the simulation.
// It is owned/updated by the simulator and movement driver.
struct DroneState {
    Pose pose;
};