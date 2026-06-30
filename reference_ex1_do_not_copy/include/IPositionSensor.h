#pragma once
// IPositionSensor.h
#include "Types.h"

// Interface for the drone's position sensor.
class IPositionSensor {
public:

    IPositionSensor() = default;
    // Virtual destructor for proper cleanup of derived classes.
    virtual ~IPositionSensor() = default;

    // Delete copy constructor and assignment operator to prevent copying of the sensor.
    IPositionSensor(const IPositionSensor&) = delete;
    IPositionSensor& operator=(const IPositionSensor&) = delete;

    // Returns the current pose of the drone as observed by the position sensor.
    virtual Pose getPose() const = 0;
};