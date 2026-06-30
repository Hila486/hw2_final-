#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/MockGPS.h>

namespace drone_mapper {
// Optional implementation for the 
class MockMovement final : public IDroneMovement {
public:
    explicit MockMovement(MockGPS& gps); // constructor that takes a MockGPS object
    // RotationDirection: Left, Right
    // HorizontalAngle: angle in degrees
    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override; // rotates the drone
    types::MovementResult advance(PhysicalLength distance) override; // advances the drone
    types::MovementResult elevate(PhysicalLength distance) override; // elevates the drone

private:
    MockGPS& gps_; // reference to the MockGPS object
};

} // namespace drone_mapper
