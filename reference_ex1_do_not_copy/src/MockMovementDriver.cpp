#include "MockMovementDriver.h"

#include <cstdlib>
#include <iostream>

namespace {
    // Helper functions to convert units and perform common calculations.
    int cmToInt(Cm value) {
        return static_cast<int>(value.numerical_value_in(cm));
    }

    int degToInt(Degree value) {
        return static_cast<int>(value.numerical_value_in(deg));
    }

    
    int absInt(int value) {
        return value < 0 ? -value : value;
    }

    int max3(int a, int b, int c) {
        int result = a;

        if (b > result) {
            result = b;
        }

        if (c > result) {
            result = c;
        }

        return result;
    }
}

/// Constructor.
MockMovementDriver::MockMovementDriver(
    DroneState& droneState,
    const DroneConfig& droneConfig,
    const GroundTruthMap& worldMap
)
    : droneState(droneState),
      droneConfig(droneConfig),
      worldMap(worldMap) {
}

/// Normalizes an angle to the range [0, 360) degrees.
Degree MockMovementDriver::normalizeAngle(Degree angle) const {
    int result = degToInt(angle) % 360;

    if (result < 0) {
        result += 360;
    }

    return result * deg;
}


//    Checks if the drone can move to a target center position.
//
//    The movement is blocked if:
//    - the target center position is outside the map
//    - the target center position is occupied
//    - any cell needed for the drone body clearance is occupied/out of bounds

bool MockMovementDriver::canMoveTo(const Position& position) const {
    int centerX = cmToInt(position.x);
    int centerY = cmToInt(position.y);
    int centerZ = cmToInt(position.height);

    int widthCm = cmToInt(droneConfig.minPassageWidth);
    int lengthCm = cmToInt(droneConfig.minPassageLength);
    int heightCm = cmToInt(droneConfig.minPassageHeight);

    int diameterCm = max3(widthCm, lengthCm, heightCm);
    int radiusCm = diameterCm / 2;

    for (int z = centerZ - radiusCm; z <= centerZ + radiusCm; ++z) {
        for (int y = centerY - radiusCm; y <= centerY + radiusCm; ++y) {
            for (int x = centerX - radiusCm; x <= centerX + radiusCm; ++x) {
                int dx = x - centerX;
                int dy = y - centerY;
                int dz = z - centerZ;

                if (dx * dx + dy * dy + dz * dz > radiusCm * radiusCm) {
                    continue;
                }

                Position checkedPosition{
                    x * cm,
                    y * cm,
                    z * cm
                };

                CellState cell = worldMap.getCell(checkedPosition);

                if (cell == CellState::OutOfBounds) {
                    std::cout << "Movement blocked: drone body would leave map near "
                              << x << ", "
                              << y << ", "
                              << z
                              << std::endl;
                    return false;
                }

                if (cell == CellState::Occupied) {
                    std::cout << "Movement blocked: drone body would collide near "
                              << x << ", "
                              << y << ", "
                              << z
                              << std::endl;
                    return false;
                }
            }
        }
    }

    return true;
}

// Rotates the drone left or right by the specified angle.
bool MockMovementDriver::rotate(RotationDirection direction, Degree angle) {
    int requestedAngle = degToInt(angle);
    int maxRotate = degToInt(droneConfig.maxRotate);

    if (absInt(requestedAngle) > maxRotate) {
        std::cout << "Rotate failed: requested angle "
                  << requestedAngle
                  << " is larger than max rotate "
                  << maxRotate
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

// Moves the drone forward or backward in the direction it is currently facing.
bool MockMovementDriver::advance(Cm distance) {
    int rawDistance = cmToInt(distance);
    int maxAdvance = cmToInt(droneConfig.maxAdvance);

    if (absInt(rawDistance) > maxAdvance) {
        std::cout << "Advance failed: requested distance "
                  << rawDistance
                  << " is larger than max advance "
                  << maxAdvance
                  << std::endl;
        return false;
    }

    if (distance == 0 * cm) {
        return true;
    }

    Degree direction = normalizeAngle(droneState.pose.xyAngle);

    int step = (rawDistance > 0) ? 1 : -1;
    int stepsCount = absInt(rawDistance);

    Position currentPosition = droneState.pose.position;

    for (int i = 0; i < stepsCount; ++i) {
        Position nextPosition = currentPosition;

        if (direction == 0 * deg) {
            nextPosition.x += step * cm;
        } else if (direction == 90 * deg) {
            nextPosition.y += step * cm;
        } else if (direction == 180 * deg) {
            nextPosition.x -= step * cm;
        } else if (direction == 270 * deg) {
            nextPosition.y -= step * cm;
        } else {
            std::cout << "Advance failed: unsupported angle "
                      << degToInt(direction)
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

// Moves the drone up or down.
bool MockMovementDriver::elevate(Cm distance) {
    int rawDistance = cmToInt(distance);
    int maxElevate = cmToInt(droneConfig.maxElevate);

    if (absInt(rawDistance) > maxElevate) {
        std::cout << "Elevate failed: requested distance "
                  << rawDistance
                  << " is larger than max elevate "
                  << maxElevate
                  << std::endl;
        return false;
    }

    if (distance == 0 * cm) {
        return true;
    }

    int step = (rawDistance > 0) ? 1 : -1;
    int stepsCount = absInt(rawDistance);

    Position currentPosition = droneState.pose.position;

    for (int i = 0; i < stepsCount; ++i) {
        Position nextPosition = currentPosition;
        nextPosition.height += step * cm;

        if (!canMoveTo(nextPosition)) {
            return false;
        }

        currentPosition = nextPosition;
    }

    droneState.pose.position = currentPosition;
    return true;
}