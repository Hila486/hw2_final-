// MockPositionSensor.cpp
#include "MockPositionSensor.h"



// Constructor 
MockPositionSensor::MockPositionSensor(const DroneState& droneState)
    : droneState(droneState) {
}

// Returns the current drone pose.
Pose MockPositionSensor::getPose() const {
    return droneState.pose;
}