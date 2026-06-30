#pragma once

#include <vector>

#include "Types.h"
#include "MissionConfig.h"
#include "DroneConfig.h"
#include "IBuildingMap.h"

enum class DronePhase {
    NeedLocation,
    NeedHorizontalScan,
    DecideAfterHorizontalScan,
    NeedUpScan,
    DecideAfterUpScan,
    NeedDownScan,
    DecideAfterDownScan,
    Backtracking,
    Finished
};

class Drone {
public:
    Drone(IBuildingMap& buildingMap,
          const MissionConfig& missionConfig,
          const DroneConfig& droneConfig);

    Command nextCommand();

    void receiveLocation(const Pose& pose);
    void receiveScanResult(const ScanResult& scan);
    void receiveMovementResult(bool success);

    bool isFinished() const;

private:
    IBuildingMap& buildingMap;
    MissionConfig missionConfig;
    DroneConfig droneConfig;

    DronePhase phase;
    bool finished;
    bool hasPose;

    Pose currentPose;
    CommandType lastCommandType;

    // Number of right rotations since last successful advance/elevate.
    // 0 = current direction, 1 = after one right turn, ..., 3 = fourth horizontal direction.
    int rotationsWithoutMoving;

    // Rotation state used when maxRotate is smaller than the desired 90 degree turn.
    // The drone may temporarily face intermediate angles, but it should not scan/advance
    // until it reaches the target cardinal direction.
    bool isTurningToTarget;
    Degree targetDirectionAngle;

    // Cells the drone has already physically visited.
    std::vector<Position> visitedPositions;

    // DFS path from the start position to the current position.
    // Used for backtracking when the drone reaches a dead end.
    std::vector<Position> pathStack;

    // Backtracking state.
    bool isBacktracking;
    bool hasBacktrackTarget;
    Position backtrackTarget;

    // Pending movement state.
    // Used so receiveMovementResult() knows whether a successful move
    // was exploration or backtracking.
    bool pendingExploreMove;
    bool pendingBacktrackMove;
    Position pendingMoveTarget;

    void markCurrentCellFree();

    void updateMapFromScan(const ScanResult& scan);
    void updateFrontCellFromScan(const ScanResult& scan);
    void updateUpCellFromScan(const ScanResult& scan);
    void updateDownCellFromScan(const ScanResult& scan);

    Position frontPosition(const Pose& pose) const;
    Position upPosition(const Pose& pose) const;
    Position downPosition(const Pose& pose) const;

    Degree normalizeAngle(Degree angle) const;
    bool inMissionBounds(const Position& p) const;

    bool wasVisited(const Position& position) const;
    void rememberVisited(const Position& position);

    bool samePosition(const Position& a, const Position& b) const;
    Degree angleToNeighbor(const Position& from, const Position& to) const;

    Command makeMoveTowardTargetCommand(const Position& target,
                                        bool backtrackMove);

    void clearPendingMove();

    int droneBodyRadiusCm() const;
    bool canDroneBodyFitAtPositionAllowUnknown(const Position& center) const;
    void markBodyCellsAtPositionFree(const Position& center);

    Command makeGetLocationCommand() const;
    Command makeScanForwardCommand() const;
    Command makeScanUpCommand() const;
    Command makeScanDownCommand() const;
    Command makeAdvanceCommand(Cm distance) const;
    Command makeElevateCommand(Cm distance) const;
    Command makeRotateTowardCommand(Degree targetAngle) const;
    Command makeFinishedCommand() const;
};