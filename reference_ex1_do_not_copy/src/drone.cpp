#include "../include/drone.h"

#include <cmath>
#include <iostream>

namespace {
    constexpr Cm STEP = SUPPORTED_RESOLUTION_CM * cm;

    int cmToInt(Cm value) {
        return static_cast<int>(value.numerical_value_in(cm));
    }

    double cmToDouble(Cm value) {
        return static_cast<double>(value.numerical_value_in(cm));
    }

    int degToInt(Degree value) {
        return static_cast<int>(value.numerical_value_in(deg));
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

Drone::Drone(IBuildingMap& buildingMap,
             const MissionConfig& missionConfig,
             const DroneConfig& droneConfig)
    : buildingMap(buildingMap),
      missionConfig(missionConfig),
      droneConfig(droneConfig),
      phase(DronePhase::NeedLocation),
      finished(false),
      hasPose(false),
      lastCommandType(CommandType::GetLocation),
      rotationsWithoutMoving(0),
      isTurningToTarget(false),
      targetDirectionAngle(0 * deg),
      isBacktracking(false),
      hasBacktrackTarget(false),
      backtrackTarget{0 * cm, 0 * cm, 0 * cm},
      pendingExploreMove(false),
      pendingBacktrackMove(false),
      pendingMoveTarget{0 * cm, 0 * cm, 0 * cm} {
}

bool Drone::isFinished() const {
    return finished;
}

Command Drone::nextCommand() {
    if (finished) {
        return makeFinishedCommand();
    }

    switch (phase) {
        case DronePhase::NeedLocation: {
            phase = DronePhase::NeedHorizontalScan;
            lastCommandType = CommandType::GetLocation;
            return makeGetLocationCommand();
        }

        case DronePhase::NeedHorizontalScan: {
            if (isTurningToTarget) {
                if (!hasPose) {
                    phase = DronePhase::NeedLocation;
                    lastCommandType = CommandType::GetLocation;
                    return makeGetLocationCommand();
                }

                if (normalizeAngle(currentPose.xyAngle) != targetDirectionAngle) {
                    phase = DronePhase::NeedLocation;
                    lastCommandType = CommandType::Rotate;
                    return makeRotateTowardCommand(targetDirectionAngle);
                }

                isTurningToTarget = false;
            }

            if (isBacktracking && hasBacktrackTarget) {
                if (!hasPose) {
                    phase = DronePhase::NeedLocation;
                    lastCommandType = CommandType::GetLocation;
                    return makeGetLocationCommand();
                }

                if (samePosition(currentPose.position, backtrackTarget)) {
                    isBacktracking = false;
                    hasBacktrackTarget = false;
                } else {
                    return makeMoveTowardTargetCommand(backtrackTarget, true);
                }
            }

            phase = DronePhase::DecideAfterHorizontalScan;
            lastCommandType = CommandType::Scan;
            return makeScanForwardCommand();
        }

        case DronePhase::DecideAfterHorizontalScan: {
            if (!hasPose) {
                phase = DronePhase::NeedLocation;
                lastCommandType = CommandType::GetLocation;
                return makeGetLocationCommand();
            }

            Position front = frontPosition(currentPose);

            if (inMissionBounds(front) &&
                buildingMap.getCell(front) != CellState::Occupied &&
                canDroneBodyFitAtPositionAllowUnknown(front) &&
                !wasVisited(front)) {

                pendingExploreMove = true;
                pendingBacktrackMove = false;
                pendingMoveTarget = front;

                phase = DronePhase::NeedLocation;
                lastCommandType = CommandType::Advance;
                return makeAdvanceCommand(STEP);
            }

            if (rotationsWithoutMoving < 3) {
                ++rotationsWithoutMoving;

                targetDirectionAngle = normalizeAngle(currentPose.xyAngle + 90 * deg);
                isTurningToTarget = true;

                phase = DronePhase::NeedLocation;
                lastCommandType = CommandType::Rotate;
                return makeRotateTowardCommand(targetDirectionAngle);
            }

            phase = DronePhase::DecideAfterUpScan;
            lastCommandType = CommandType::Scan;
            return makeScanUpCommand();
        }

        case DronePhase::NeedUpScan: {
            phase = DronePhase::DecideAfterUpScan;
            lastCommandType = CommandType::Scan;
            return makeScanUpCommand();
        }

        case DronePhase::DecideAfterUpScan: {
            if (!hasPose) {
                phase = DronePhase::NeedLocation;
                lastCommandType = CommandType::GetLocation;
                return makeGetLocationCommand();
            }

            Position up = upPosition(currentPose);

            if (inMissionBounds(up) &&
                buildingMap.getCell(up) != CellState::Occupied &&
                canDroneBodyFitAtPositionAllowUnknown(up) &&
                !wasVisited(up)) {

                pendingExploreMove = true;
                pendingBacktrackMove = false;
                pendingMoveTarget = up;

                phase = DronePhase::NeedLocation;
                lastCommandType = CommandType::Elevate;
                return makeElevateCommand(STEP);
            }

            phase = DronePhase::DecideAfterDownScan;
            lastCommandType = CommandType::Scan;
            return makeScanDownCommand();
        }

        case DronePhase::NeedDownScan: {
            phase = DronePhase::DecideAfterDownScan;
            lastCommandType = CommandType::Scan;
            return makeScanDownCommand();
        }

        case DronePhase::DecideAfterDownScan: {
            if (!hasPose) {
                phase = DronePhase::NeedLocation;
                lastCommandType = CommandType::GetLocation;
                return makeGetLocationCommand();
            }

            Position down = downPosition(currentPose);

            if (inMissionBounds(down) &&
                buildingMap.getCell(down) != CellState::Occupied &&
                canDroneBodyFitAtPositionAllowUnknown(down) &&
                !wasVisited(down)) {

                pendingExploreMove = true;
                pendingBacktrackMove = false;
                pendingMoveTarget = down;

                phase = DronePhase::NeedLocation;
                lastCommandType = CommandType::Elevate;
                return makeElevateCommand(-STEP);
            }

            if (pathStack.size() <= 1) {
                finished = true;
                phase = DronePhase::Finished;
                lastCommandType = CommandType::Finished;
                return makeFinishedCommand();
            }

            backtrackTarget = pathStack[pathStack.size() - 2];
            hasBacktrackTarget = true;
            isBacktracking = true;

            return makeMoveTowardTargetCommand(backtrackTarget, true);
        }

        case DronePhase::Backtracking: {
            if (!hasBacktrackTarget) {
                if (pathStack.size() <= 1) {
                    finished = true;
                    phase = DronePhase::Finished;
                    lastCommandType = CommandType::Finished;
                    return makeFinishedCommand();
                }

                backtrackTarget = pathStack[pathStack.size() - 2];
                hasBacktrackTarget = true;
                isBacktracking = true;
            }

            return makeMoveTowardTargetCommand(backtrackTarget, true);
        }

        case DronePhase::Finished: {
            finished = true;
            lastCommandType = CommandType::Finished;
            return makeFinishedCommand();
        }
    }

    finished = true;
    return makeFinishedCommand();
}

void Drone::receiveLocation(const Pose& pose) {
    currentPose = pose;
    hasPose = true;

    markCurrentCellFree();

    /*
        If the drone is physically standing here, then its body footprint
        should be considered free in the drone's internal map.

        This helps the drone understand that the current occupied body space
        is safe, not just the center cell.
    */
    markBodyCellsAtPositionFree(currentPose.position);

    if (pathStack.empty()) {
        pathStack.push_back(currentPose.position);
    }
}

void Drone::receiveScanResult(const ScanResult& scan) {
    if (!hasPose) {
        return;
    }

    updateMapFromScan(scan);

    if (phase == DronePhase::DecideAfterHorizontalScan) {
        updateFrontCellFromScan(scan);
    } else if (phase == DronePhase::DecideAfterUpScan) {
        updateUpCellFromScan(scan);
    } else if (phase == DronePhase::DecideAfterDownScan) {
        updateDownCellFromScan(scan);
    }
}

void Drone::receiveMovementResult(bool success) {
    if (!success) {
        /*
            A failed movement does NOT necessarily mean the target center cell
            is occupied.

            It may fail because the drone body would collide near the target,
            or because the drone body would leave the map.

            So we do NOT mark pendingMoveTarget as Occupied.
            We only remember it as attempted, to avoid retrying forever.
        */
        bool failedAdvance = (lastCommandType == CommandType::Advance);
        bool failedElevate = (lastCommandType == CommandType::Elevate);

        bool failedUpMove =
            failedElevate &&
            hasPose &&
            pendingMoveTarget.height > currentPose.position.height;

        bool failedDownMove =
            failedElevate &&
            hasPose &&
            pendingMoveTarget.height < currentPose.position.height;

        if ((failedAdvance || failedElevate) &&
            inMissionBounds(pendingMoveTarget)) {
            rememberVisited(pendingMoveTarget);
        }

        clearPendingMove();

        isBacktracking = false;
        hasBacktrackTarget = false;

        if (failedAdvance) {
            phase = DronePhase::DecideAfterHorizontalScan;
        } else if (failedUpMove) {
            phase = DronePhase::DecideAfterUpScan;
        } else if (failedDownMove) {
            phase = DronePhase::DecideAfterDownScan;
        } else {
            phase = DronePhase::NeedLocation;
        }

        return;
    }

    if (lastCommandType == CommandType::Advance ||
        lastCommandType == CommandType::Elevate) {

        if (pendingExploreMove) {
            if (pathStack.empty() ||
                !samePosition(pathStack.back(), pendingMoveTarget)) {
                pathStack.push_back(pendingMoveTarget);
            }
        }

        if (pendingBacktrackMove) {
            if (pathStack.size() > 1) {
                pathStack.pop_back();
            }

            isBacktracking = false;
            hasBacktrackTarget = false;
        }

        rotationsWithoutMoving = 0;
        clearPendingMove();
    }

    phase = DronePhase::NeedLocation;
}

bool Drone::wasVisited(const Position& position) const {
    for (const Position& visited : visitedPositions) {
        if (samePosition(visited, position)) {
            return true;
        }
    }

    return false;
}

void Drone::rememberVisited(const Position& position) {
    if (!inMissionBounds(position)) {
        return;
    }

    if (wasVisited(position)) {
        return;
    }

    visitedPositions.push_back(position);
}

bool Drone::samePosition(const Position& a, const Position& b) const {
    return a.x == b.x &&
           a.y == b.y &&
           a.height == b.height;
}

void Drone::clearPendingMove() {
    pendingExploreMove = false;
    pendingBacktrackMove = false;
    pendingMoveTarget = Position{0 * cm, 0 * cm, 0 * cm};
}

int Drone::droneBodyRadiusCm() const {
    int widthCm = cmToInt(droneConfig.minPassageWidth);
    int lengthCm = cmToInt(droneConfig.minPassageLength);
    int heightCm = cmToInt(droneConfig.minPassageHeight);

    int diameterCm = max3(widthCm, lengthCm, heightCm);

    if (diameterCm <= 0) {
        return 0;
    }

    return diameterCm / 2;
}

bool Drone::canDroneBodyFitAtPositionAllowUnknown(const Position& center) const {
    int centerX = cmToInt(center.x);
    int centerY = cmToInt(center.y);
    int centerZ = cmToInt(center.height);

    int radiusCm = droneBodyRadiusCm();

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

                if (!inMissionBounds(checkedPosition)) {
                    return false;
                }

                CellState cell = buildingMap.getCell(checkedPosition);

                /*
                    Unknown cells are allowed for an attempted movement.

                    The drone should not use the hidden real map.
                    It only avoids cells that its internal map already knows
                    are occupied. The MockMovementDriver will decide whether
                    the movement is physically possible in the real world.
                */
                if (cell == CellState::Occupied) {
                    return false;
                }
            }
        }
    }

    return true;
}

void Drone::markBodyCellsAtPositionFree(const Position& center) {
    int centerX = cmToInt(center.x);
    int centerY = cmToInt(center.y);
    int centerZ = cmToInt(center.height);

    int radiusCm = droneBodyRadiusCm();

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

                if (!inMissionBounds(checkedPosition)) {
                    continue;
                }

                if (buildingMap.getCell(checkedPosition) != CellState::Occupied) {
                    buildingMap.setCell(checkedPosition, CellState::Free);
                }
            }
        }
    }
}

void Drone::markCurrentCellFree() {
    if (inMissionBounds(currentPose.position)) {
        buildingMap.setCell(currentPose.position, CellState::Free);
        rememberVisited(currentPose.position);
    }
}

/*
    General lidar map update.

    If distance > 0:
        - mark cells before the hit as Free
        - mark the hit cell as Occupied

    If distance == 0:
        - skip here
        - because it means object is closer than Z-min, exact cell unknown
*/
void Drone::updateMapFromScan(const ScanResult& scan) {
    const double pi = 3.14159265358979323846;

    for (const ScanHit& hit : scan) {
        if (hit.distance <= 0 * cm) {
            continue;
        }

        Degree absoluteXyAngle =
            normalizeAngle(currentPose.xyAngle + hit.angle.xyAngle);

        double xyRadians =
            static_cast<double>(degToInt(absoluteXyAngle)) * pi / 180.0;

        double heightRadians =
            static_cast<double>(degToInt(hit.angle.heightAngle)) * pi / 180.0;

        double horizontalFactor = std::cos(heightRadians);

        double dx = horizontalFactor * std::cos(xyRadians);
        double dy = horizontalFactor * std::sin(xyRadians);
        double dh = std::sin(heightRadians);

        int maxDistanceCm = cmToInt(hit.distance);

        int lastFreeDistanceCm = maxDistanceCm;

        if (hit.hitObject) {
            lastFreeDistanceCm = maxDistanceCm - SUPPORTED_RESOLUTION_CM;
        }

        for (int distanceCm = SUPPORTED_RESOLUTION_CM;
             distanceCm <= lastFreeDistanceCm;
             distanceCm += SUPPORTED_RESOLUTION_CM) {

            Position freePosition{
                static_cast<int>(
                    std::round(
                        cmToDouble(currentPose.position.x) +
                        dx * static_cast<double>(distanceCm)
                    )
                ) * cm,

                static_cast<int>(
                    std::round(
                        cmToDouble(currentPose.position.y) +
                        dy * static_cast<double>(distanceCm)
                    )
                ) * cm,

                static_cast<int>(
                    std::round(
                        cmToDouble(currentPose.position.height) +
                        dh * static_cast<double>(distanceCm)
                    )
                ) * cm
            };

            if (!inMissionBounds(freePosition)) {
                continue;
            }

            if (samePosition(freePosition, currentPose.position)) {
                continue;
            }

            if (buildingMap.getCell(freePosition) != CellState::Occupied) {
                buildingMap.setCell(freePosition, CellState::Free);
            }
        }

        if (hit.hitObject) {
            Position hitPosition{
                static_cast<int>(
                    std::round(
                        cmToDouble(currentPose.position.x) +
                        dx * static_cast<double>(maxDistanceCm)
                    )
                ) * cm,

                static_cast<int>(
                    std::round(
                        cmToDouble(currentPose.position.y) +
                        dy * static_cast<double>(maxDistanceCm)
                    )
                ) * cm,

                static_cast<int>(
                    std::round(
                        cmToDouble(currentPose.position.height) +
                        dh * static_cast<double>(maxDistanceCm)
                    )
                ) * cm
            };

            std::cout << "LIDAR occupied hit: "
                      << "drone=("
                      << cmToInt(currentPose.position.x) << ", "
                      << cmToInt(currentPose.position.y) << ", "
                      << cmToInt(currentPose.position.height) << "), "
                      << "droneAngle=" << degToInt(currentPose.xyAngle) << ", "
                      << "relativeAngle=("
                      << degToInt(hit.angle.xyAngle) << ", "
                      << degToInt(hit.angle.heightAngle) << "), "
                      << "distance=" << cmToInt(hit.distance) << ", "
                      << "hitCell=("
                      << cmToInt(hitPosition.x) << ", "
                      << cmToInt(hitPosition.y) << ", "
                      << cmToInt(hitPosition.height) << ")"
                      << std::endl;

            if (inMissionBounds(hitPosition)) {
                buildingMap.setCell(hitPosition, CellState::Occupied);
            }
        }
    }
}

void Drone::updateFrontCellFromScan(const ScanResult& scan) {
    Position front = frontPosition(currentPose);

    if (!inMissionBounds(front)) {
        return;
    }

    if (buildingMap.getCell(front) == CellState::Occupied) {
        return;
    }

    for (const ScanHit& hit : scan) {
        if (!hit.hitObject) {
            continue;
        }

        bool isCentralForwardBeam =
            hit.angle.xyAngle == 0 * deg &&
            hit.angle.heightAngle == 0 * deg;

        if (!isCentralForwardBeam) {
            continue;
        }

        if (hit.distance <= 0 * cm) {
            /*
                Too close to measure accurately.
                Keep map as Unknown, but do not move there.
            */
            rememberVisited(front);
            return;
        }

        if (hit.distance <= STEP) {
            buildingMap.setCell(front, CellState::Occupied);
            return;
        }
    }

    buildingMap.setCell(front, CellState::Free);
}

void Drone::updateUpCellFromScan(const ScanResult& scan) {
    Position up = upPosition(currentPose);

    if (!inMissionBounds(up)) {
        return;
    }

    if (buildingMap.getCell(up) == CellState::Occupied) {
        return;
    }

    for (const ScanHit& hit : scan) {
        if (!hit.hitObject) {
            continue;
        }

        bool isCentralUpBeam =
            hit.angle.xyAngle == 0 * deg &&
            hit.angle.heightAngle == 90 * deg;

        if (!isCentralUpBeam) {
            continue;
        }

        if (hit.distance <= 0 * cm) {
            rememberVisited(up);
            return;
        }

        if (hit.distance <= STEP) {
            buildingMap.setCell(up, CellState::Occupied);
            return;
        }
    }

    buildingMap.setCell(up, CellState::Free);
}

void Drone::updateDownCellFromScan(const ScanResult& scan) {
    Position down = downPosition(currentPose);

    if (!inMissionBounds(down)) {
        return;
    }

    if (buildingMap.getCell(down) == CellState::Occupied) {
        return;
    }

    for (const ScanHit& hit : scan) {
        if (!hit.hitObject) {
            continue;
        }

        bool isCentralDownBeam =
            hit.angle.xyAngle == 0 * deg &&
            hit.angle.heightAngle == (-90) * deg;

        if (!isCentralDownBeam) {
            continue;
        }

        if (hit.distance <= 0 * cm) {
            rememberVisited(down);
            return;
        }

        if (hit.distance <= STEP) {
            buildingMap.setCell(down, CellState::Occupied);
            return;
        }
    }

    buildingMap.setCell(down, CellState::Free);
}

/*
    Assignment convention:
    0   = east  -> x + 1
    90  = south -> y + 1
    180 = west  -> x - 1
    270 = north -> y - 1
*/
Position Drone::frontPosition(const Pose& pose) const {
    Degree angle = normalizeAngle(pose.xyAngle);

    if (angle == 0 * deg) {
        return Position{
            pose.position.x + STEP,
            pose.position.y,
            pose.position.height
        };
    }

    if (angle == 90 * deg) {
        return Position{
            pose.position.x,
            pose.position.y + STEP,
            pose.position.height
        };
    }

    if (angle == 180 * deg) {
        return Position{
            pose.position.x - STEP,
            pose.position.y,
            pose.position.height
        };
    }

    if (angle == 270 * deg) {
        return Position{
            pose.position.x,
            pose.position.y - STEP,
            pose.position.height
        };
    }

    return pose.position;
}

Position Drone::upPosition(const Pose& pose) const {
    return Position{
        pose.position.x,
        pose.position.y,
        pose.position.height + STEP
    };
}

Position Drone::downPosition(const Pose& pose) const {
    return Position{
        pose.position.x,
        pose.position.y,
        pose.position.height - STEP
    };
}

Degree Drone::angleToNeighbor(const Position& from, const Position& to) const {
    if (to.x == from.x + STEP &&
        to.y == from.y &&
        to.height == from.height) {
        return 0 * deg;
    }

    if (to.x == from.x &&
        to.y == from.y + STEP &&
        to.height == from.height) {
        return 90 * deg;
    }

    if (to.x == from.x - STEP &&
        to.y == from.y &&
        to.height == from.height) {
        return 180 * deg;
    }

    if (to.x == from.x &&
        to.y == from.y - STEP &&
        to.height == from.height) {
        return 270 * deg;
    }

    return normalizeAngle(currentPose.xyAngle);
}

Command Drone::makeMoveTowardTargetCommand(
    const Position& target,
    bool backtrackMove
) {
    if (!hasPose) {
        phase = DronePhase::NeedLocation;
        lastCommandType = CommandType::GetLocation;
        return makeGetLocationCommand();
    }

    if (target.height > currentPose.position.height) {
        pendingExploreMove = !backtrackMove;
        pendingBacktrackMove = backtrackMove;
        pendingMoveTarget = target;

        phase = DronePhase::NeedLocation;
        lastCommandType = CommandType::Elevate;
        return makeElevateCommand(STEP);
    }

    if (target.height < currentPose.position.height) {
        pendingExploreMove = !backtrackMove;
        pendingBacktrackMove = backtrackMove;
        pendingMoveTarget = target;

        phase = DronePhase::NeedLocation;
        lastCommandType = CommandType::Elevate;
        return makeElevateCommand(-STEP);
    }

    Degree desiredAngle = angleToNeighbor(currentPose.position, target);
    Degree currentAngle = normalizeAngle(currentPose.xyAngle);

    if (currentAngle != desiredAngle) {
        phase = DronePhase::NeedLocation;
        lastCommandType = CommandType::Rotate;
        return makeRotateTowardCommand(desiredAngle);
    }

    pendingExploreMove = !backtrackMove;
    pendingBacktrackMove = backtrackMove;
    pendingMoveTarget = target;

    phase = DronePhase::NeedLocation;
    lastCommandType = CommandType::Advance;
    return makeAdvanceCommand(STEP);
}

Degree Drone::normalizeAngle(Degree angle) const {
    int rawAngle = degToInt(angle);

    rawAngle = rawAngle % 360;

    if (rawAngle < 0) {
        rawAngle += 360;
    }

    return rawAngle * deg;
}

bool Drone::inMissionBounds(const Position& p) const {
    return p.x >= missionConfig.minX &&
           p.x <= missionConfig.maxX &&
           p.y >= missionConfig.minY &&
           p.y <= missionConfig.maxY &&
           p.height >= missionConfig.minZ &&
           p.height <= missionConfig.maxZ;
}

Command Drone::makeGetLocationCommand() const {
    Command command{};
    command.type = CommandType::GetLocation;
    return command;
}

Command Drone::makeScanForwardCommand() const {
    Command command{};
    command.type = CommandType::Scan;
    command.scanAngle = ScanAngle(0 * deg, 0 * deg);
    return command;
}

Command Drone::makeScanUpCommand() const {
    Command command{};
    command.type = CommandType::Scan;
    command.scanAngle = ScanAngle(0 * deg, 90 * deg);
    return command;
}

Command Drone::makeScanDownCommand() const {
    Command command{};
    command.type = CommandType::Scan;
    command.scanAngle = ScanAngle(0 * deg, (-90) * deg);
    return command;
}

Command Drone::makeAdvanceCommand(Cm distance) const {
    Command command{};
    command.type = CommandType::Advance;
    command.distance = distance;
    return command;
}

Command Drone::makeElevateCommand(Cm distance) const {
    Command command{};
    command.type = CommandType::Elevate;
    command.distance = distance;
    return command;
}

Command Drone::makeRotateTowardCommand(Degree targetAngle) const {
    Command command{};
    command.type = CommandType::Rotate;

    Degree currentAngle = normalizeAngle(currentPose.xyAngle);
    Degree normalizedTarget = normalizeAngle(targetAngle);

    int current = degToInt(currentAngle);
    int target = degToInt(normalizedTarget);
    int maxRotate = degToInt(droneConfig.maxRotate);

    int clockwiseDiff = (target - current + 360) % 360;

    if (clockwiseDiff == 0) {
        command.rotationDirection = RotationDirection::Right;
        command.angle = 0 * deg;
        return command;
    }

    int signedDiff = clockwiseDiff;
    if (signedDiff > 180) {
        signedDiff -= 360;
    }

    int remaining = signedDiff < 0 ? -signedDiff : signedDiff;
    int step = remaining;

    if (maxRotate > 0 && step > maxRotate) {
        step = maxRotate;
    }

    command.rotationDirection =
        (signedDiff >= 0) ? RotationDirection::Right : RotationDirection::Left;

    command.angle = step * deg;

    return command;
}

Command Drone::makeFinishedCommand() const {
    Command command{};
    command.type = CommandType::Finished;
    return command;
}