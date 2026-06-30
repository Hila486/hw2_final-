#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

// **Do not change this interface.**
class IDroneMovement {
public:
    virtual ~IDroneMovement() = default; // virtual destructor

    virtual types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) = 0;// rotates the drone
    virtual types::MovementResult advance(PhysicalLength distance) = 0;// advances the drone
    virtual types::MovementResult elevate(PhysicalLength distance) = 0; // Can be negative! elevates the drone
};

} // namespace drone_mapper
