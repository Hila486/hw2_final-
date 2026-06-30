#include "MockMovementDriver.h"

#include <cstdlib>
#include <iostream>

/*
    Constructor.

    We store references, not copies.

    This is important because when this class updates droneState,
    the simulator and sensors should see the updated position too.
*/
MockMovementDriver::MockMovementDriver(
    DroneState& droneState,
    const DroneConfig& droneConfig,
    const GroundTruthMap& worldMap
)
    : droneState(droneState),
      droneConfig(droneConfig),
      worldMap(worldMap) {
}

/*
    Normalize angle to the range [0, 359].
*/
Degree MockMovementDriver::normalizeAngle(Degree angle) const {
    Degree result = angle % 360;

    if (result < 0) {
        result += 360;
    }

    return result;
}

/*
    Checks if a position is legal for the drone.

    The drone cannot move:
    - outside the map
    - into an occupied cell
*/
bool MockMovementDriver::canMoveTo(const Position& position) const {
    CellState cell = worldMap.getCell(position);

    if (cell == CellState::OutOfBounds) {
        std::cout << "Movement blocked: target position is out of bounds at "
                  << position.x << ", "
                  << position.y << ", "
                  << position.height
                  << std::endl;
        return false;
    }

    if (cell == CellState::Occupied) {
        std::cout << "Movement blocked: target position is occupied at "
                  << position.x << ", "
                  << position.y << ", "
                  << position.height
                  << std::endl;
        return false;
    }

    return true;
}

/*
    Rotates the drone.

    Angle convention from the assignment:
    0   = east
    90  = south
    180 = west
    270 = north

    Therefore:
    - turning right increases the angle
    - turning left decreases the angle
*/
bool MockMovementDriver::rotate(RotationDirection direction, Degree angle) {
    if (std::abs(angle) > droneConfig.maxRotateDeg) {
        std::cout << "Rotate failed: requested angle "
                  << angle
                  << " is larger than max rotate "
                  << droneConfig.maxRotateDeg
                  << std::endl;
        return false;
    }

    Degree signedAngle = angle;

    if (direction == RotationDirection::Left) {
        signedAngle = -signedAngle;
    }

    droneState.pose.xyAngle =
        normalizeAngle(droneState.pose.xyAngle + signedAngle);

    return true;
}

/*
    Advances the drone forward/backward.

    Important improvement:
    This version checks every intermediate cell, not only the final cell.

    Example:
    If the drone moves from x=0 to x=5, we check:
    x=1, x=2, x=3, x=4, x=5

    This prevents the drone from jumping through walls.
*/
bool MockMovementDriver::advance(Cm distance) {
    if (std::abs(distance) > droneConfig.maxAdvanceCm) {
        std::cout << "Advance failed: requested distance "
                  << distance
                  << " is larger than max advance "
                  << droneConfig.maxAdvanceCm
                  << std::endl;
        return false;
    }

    if (distance == 0) {
        return true;
    }

    Degree direction = normalizeAngle(droneState.pose.xyAngle);

    int step = (distance > 0) ? 1 : -1;
    int stepsCount = std::abs(distance);

    Position currentPosition = droneState.pose.position;

    for (int i = 0; i < stepsCount; ++i) {
        Position nextPosition = currentPosition;

        if (direction == 0) {
            nextPosition.x += step;
        } else if (direction == 90) {
            nextPosition.y += step;
        } else if (direction == 180) {
            nextPosition.x -= step;
        } else if (direction == 270) {
            nextPosition.y -= step;
        } else {
            std::cout << "Advance failed: unsupported angle "
                      << direction
                      << ". Current simple movement supports only 0, 90, 180, 270."
                      << std::endl;
            return false;
        }

        if (!canMoveTo(nextPosition)) {
            return false;
        }

        currentPosition = nextPosition;
    }

    droneState.pose.position = currentPosition;
    return true;
}

/*
    Elevates the drone up/down.

    Positive distance increases height.
    Negative distance decreases height.

    Important improvement:
    This version checks every intermediate height layer,
    not only the final height.
*/
bool MockMovementDriver::elevate(Cm distance) {
    if (std::abs(distance) > droneConfig.maxElevateCm) {
        std::cout << "Elevate failed: requested distance "
                  << distance
                  << " is larger than max elevate "
                  << droneConfig.maxElevateCm
                  << std::endl;
        return false;
    }

    if (distance == 0) {
        return true;
    }

    int step = (distance > 0) ? 1 : -1;
    int stepsCount = std::abs(distance);

    Position currentPosition = droneState.pose.position;

    for (int i = 0; i < stepsCount; ++i) {
        Position nextPosition = currentPosition;
        nextPosition.height += step;

        if (!canMoveTo(nextPosition)) {
            return false;
        }

        currentPosition = nextPosition;
    }

    droneState.pose.position = currentPosition;
    return true;
}