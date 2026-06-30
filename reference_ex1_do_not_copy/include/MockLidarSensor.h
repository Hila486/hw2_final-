#pragma once

#include "ILidarSensor.h"
#include "IPositionSensor.h"
#include "GroundTruthMap.h"
#include "DroneConfig.h"

// this class simulates the drone's lidar sensor by tracing beams through the hidden world map.
class MockLidarSensor : public ILidarSensor {
public:
// Constructor.
    MockLidarSensor(
        const DroneConfig& droneConfig,
        const GroundTruthMap& worldMap,
        const IPositionSensor& positionSensor
    );
    // Virtual destructor for proper cleanup of derived classes.
    MockLidarSensor(const MockLidarSensor&) = delete;
    // Delete copy constructor and assignment operator to prevent copying of the sensor.
    MockLidarSensor& operator=(const MockLidarSensor&) = delete;
    ~MockLidarSensor() override = default;

    // Performs a lidar scan at the given scan angle, and returns a list of scan hits.
    ScanResult scan(const ScanAngle& scanAngle) const override;

private:
// References to the drone configuration, world map, and position sensor.
    const DroneConfig& droneConfig;
    const GroundTruthMap& worldMap;
    const IPositionSensor& positionSensor;

    // Normalizes an angle to the range [0, 360) degrees.
    Degree normalizeAngle(Degree angle) const;

    // Traces a single lidar beam from the drone's current position at the specified angles.
    bool traceBeam(
        Degree absoluteXyAngle,
        Degree heightAngle,
        Cm& hitDistance
    ) const;
};