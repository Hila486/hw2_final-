#pragma once
// MockMovementDriver.h

#include "IMovementDriver.h"
#include "DroneState.h"
#include "DroneConfig.h"
#include "GroundTruthMap.h"

//This class simulates the drone movement engine.
class MockMovementDriver : public IMovementDriver {
public:
    //Constructor.
    //droneState -  The real current state of the drone in the simulator.
    //droneConfig -  Contains movement limits and drone size/clearance settings.
    //worldMap -  The hidden real building map.

    MockMovementDriver(
        DroneState& droneState,
        const DroneConfig& droneConfig,
        const GroundTruthMap& worldMap
    );


    ~MockMovementDriver() override = default;

    // Prevent copying.
    // This class stores references to simulator objects.
    MockMovementDriver(const MockMovementDriver&) = delete;
    MockMovementDriver& operator=(const MockMovementDriver&) = delete;

    // Prevent moving for the same reason.
    MockMovementDriver(MockMovementDriver&&) = delete;
    MockMovementDriver& operator=(MockMovementDriver&&) = delete;


    // Rotates the drone left or right by the specified angle.
    bool rotate(RotationDirection direction, Degree angle) override;

    // Moves the drone forward or backward in the direction it is currently facing.
    bool advance(Cm distance) override;

    // Moves the drone up or down.
    bool elevate(Cm distance) override;

private:

    // References to the drone state, configuration, and world map.
    DroneState& droneState;
    const DroneConfig& droneConfig;
    const GroundTruthMap& worldMap;

    // Normalizes an angle to the range [0, 360) degrees.
    Degree normalizeAngle(Degree angle) const;

    // Checks if the drone can move to the specified position without colliding with obstacles or exceeding movement limits.
    bool canMoveTo(const Position& position) const;
};