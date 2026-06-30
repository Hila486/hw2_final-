#pragma once
// ILidarSensor.h
#include "Types.h"

// Interface for the drone's lidar sensor.
#pragma once

#include "Types.h"

// Interface for the drone's lidar sensor.
class ILidarSensor {
public:
    //virtual destructor for proper cleanup of derived classes.
    virtual ~ILidarSensor() = default;

    // Delete copy constructor and assignment operator to prevent copying of the sensor.
    ILidarSensor(const ILidarSensor&) = delete;
    ILidarSensor& operator=(const ILidarSensor&) = delete;
    // Performs a lidar scan at the given scan angle, and returns a list of scan hits.
    virtual ScanResult scan(const ScanAngle& scanAngle) const = 0;

protected:
    // Protected default constructor to allow construction of derived classes, but prevent direct instantiation of the interface.
    ILidarSensor() = default;
};