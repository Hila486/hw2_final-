#include "Simulator.h"
#include <iostream>
#include "../include/Units.h"

#include "ConfigParser.h"
#include "Drone.h"
#include "DroneState.h"
#include "MockLidarSensor.h"
#include "MockMovementDriver.h"
#include "MockPositionSensor.h"
#include "SparseBuildingMap.h"
#include "MapFileWriter.h"
#include "ScoreCalculator.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>

namespace {
    // Helper functions to convert units and perform common calculations.
    int cmToInt(Cm value) {
        return static_cast<int>(value.numerical_value_in(cm));
    }

    int droneRadiusCm(const DroneConfig& droneConfig) {
        int width = cmToInt(droneConfig.minPassageWidth);
        int length = cmToInt(droneConfig.minPassageLength);
        int height = cmToInt(droneConfig.minPassageHeight);

        int diameter = std::max({width, length, height});

        return diameter / 2;
    }
    // Helper method to check if a position is within the bounds of the mission area defined in the mission configuration.
    bool isInitialDroneStateValid(
        const MissionConfig& missionConfig,
        const DroneConfig& droneConfig,
        const GroundTruthMap& worldMap
    ) {
        const Position& center = missionConfig.startPosition;

        int centerX = cmToInt(center.x);
        int centerY = cmToInt(center.y);
        int centerZ = cmToInt(center.height);

        int minX = cmToInt(missionConfig.minX);
        int maxX = cmToInt(missionConfig.maxX);
        int minY = cmToInt(missionConfig.minY);
        int maxY = cmToInt(missionConfig.maxY);
        int minZ = cmToInt(missionConfig.minZ);
        int maxZ = cmToInt(missionConfig.maxZ);

        int radius = droneRadiusCm(droneConfig);

        /*
            1. Check that the drone's spherical body with the given radius
            can fit inside the mission boundaries when centered at the start position.
        */
        if (centerX - radius < minX ||
            centerX + radius > maxX ||
            centerY - radius < minY ||
            centerY + radius > maxY ||
            centerZ - radius < minZ ||
            centerZ + radius > maxZ) {

            std::cout << "Invalid initial position: drone body does not fit inside mission boundaries."
                      << std::endl;
            return false;
        }

        /*
            2. Check that every cell occupied by the drone's spherical body
            is inside the real map and is not occupied.
        */
        for (int z = centerZ - radius; z <= centerZ + radius; ++z) {
            for (int y = centerY - radius; y <= centerY + radius; ++y) {
                for (int x = centerX - radius; x <= centerX + radius; ++x) {
                    int dx = x - centerX;
                    int dy = y - centerY;
                    int dz = z - centerZ;

                    bool insideDroneSphere =
                        dx * dx + dy * dy + dz * dz <= radius * radius;

                    if (!insideDroneSphere) {
                        continue;
                    }

                    Position checkedPosition{
                        x * cm,
                        y * cm,
                        z * cm
                    };

                    CellState cell = worldMap.getCell(checkedPosition);

                    if (cell == CellState::OutOfBounds) {
                        std::cout << "Invalid initial position: part of the drone is outside the map."
                                  << std::endl;
                        return false;
                    }

                    if (cell == CellState::Occupied) {
                        std::cout << "Invalid initial position: part of the drone overlaps an occupied cell."
                                  << std::endl;
                        return false;
                    }
                }
            }
        }

        return true;
    }

    // Helper method to write a row to the drone log CSV file, including the current step number, event description, and the drone's current state (position and angle).
    void writeLogRow(
        std::ofstream& logFile,
        int step,
        const std::string& event,
        const DroneState& droneState
    ) {
        if (!logFile.is_open()) {
            return;
        }

        logFile << step << ","
                << event << ","
                << droneState.pose.position.x << ","
                << droneState.pose.position.y << ","
                << droneState.pose.position.height << ","
                << droneState.pose.xyAngle
                << "\n";
    }
}

// -----------------------------------------------------
Simulator::Simulator(const std::string& inputOutputPath)
    : inputOutputPath(inputOutputPath) {
}

std::string Simulator::makePath(const std::string& fileName) const {
    if (inputOutputPath.empty() || inputOutputPath == ".") {
        return "./" + fileName;
    }

    if (inputOutputPath.back() == '/') {
        return inputOutputPath + fileName;
    }

    return inputOutputPath + "/" + fileName;
}

int Simulator::run() {
    ConfigParser::clearInputErrors();

    std::string droneConfigPath = makePath("drone_config.txt");
    std::string missionConfigPath = makePath("mission_config.txt");
    std::string mapInputPath = makePath("map_input.txt");
    std::string mapOutputPath = makePath("map_output.txt");
    std::string inputErrorsPath = makePath("input_errors.txt");
    std::string droneLogPath = makePath("drone_log.csv");

    DroneConfig droneConfig =
        ConfigParser::parseDroneConfig(droneConfigPath);

    MissionConfig missionConfig =
        ConfigParser::parseMissionConfig(missionConfigPath);

    GroundTruthMap worldMap =
        ConfigParser::parseMapInput(mapInputPath);

    ConfigParser::writeInputErrors(inputErrorsPath);

    if (!isInitialDroneStateValid(missionConfig, droneConfig, worldMap)) {
        std::cout << "Simulation stopped: invalid initial drone state."
                  << std::endl;
        return 1;
    }

    std::cout << "Mission start position: "
              << missionConfig.startPosition.x << ", "
              << missionConfig.startPosition.y << ", "
              << missionConfig.startPosition.height
              << std::endl;

    DroneState droneState;
    droneState.pose = Pose{
        missionConfig.startPosition,
        missionConfig.startAngleDeg
    };

    std::ofstream logFile(droneLogPath);

    if (logFile.is_open()) {
        logFile << "step,event,x,y,height,angle\n";
        writeLogRow(logFile, 0, "start", droneState);
    }

    MockPositionSensor positionSensor(droneState);
    MockMovementDriver movementDriver(droneState, droneConfig, worldMap);
    MockLidarSensor lidarSensor(droneConfig, worldMap, positionSensor);

    SparseBuildingMap droneMap(
        worldMap.getSizeX(),
        worldMap.getSizeY(),
        worldMap.getSizeZ()
    );

    Drone drone(droneMap, missionConfig, droneConfig);

    const int maxIterations = 100000;

    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        Command command = drone.nextCommand();

        switch (command.type) {
            case CommandType::GetLocation: {
                Pose pose = positionSensor.getPose();
                drone.receiveLocation(pose);

                writeLogRow(
                    logFile,
                    iteration + 1,
                    "get_location",
                    droneState
                );

                break;
            }

            case CommandType::Scan: {
                std::cout << "SCAN command: "
                << "drone=("
                << static_cast<int>(droneState.pose.position.x.numerical_value_in(cm)) << ", "
                << static_cast<int>(droneState.pose.position.y.numerical_value_in(cm)) << ", "
                << static_cast<int>(droneState.pose.position.height.numerical_value_in(cm)) << "), "
                << "droneAngle=" << static_cast<int>(droneState.pose.xyAngle.numerical_value_in(deg)) << ", "
                << "scanXY=" << static_cast<int>(command.scanAngle.xyAngle.numerical_value_in(deg)) << ", "
                << "scanHeight=" << static_cast<int>(command.scanAngle.heightAngle.numerical_value_in(deg))
                << std::endl;

                ScanResult scanResult = lidarSensor.scan(command.scanAngle);
                drone.receiveScanResult(scanResult);

                writeLogRow(
                    logFile,
                    iteration + 1,
                    "scan",
                    droneState
                );

                 break;
            }

            case CommandType::Rotate: {
                bool success = movementDriver.rotate(
                    command.rotationDirection,
                    command.angle
                );

                drone.receiveMovementResult(success);

                writeLogRow(
                    logFile,
                    iteration + 1,
                    success ? "rotate_success" : "rotate_failed",
                    droneState
                );

                if (!success) {
                    std::cout << "Rotate failed; drone stayed in place."
                              << std::endl;
                }

                break;
            }

            case CommandType::Advance: {
                bool success = movementDriver.advance(command.distance);

                drone.receiveMovementResult(success);

                writeLogRow(
                    logFile,
                    iteration + 1,
                    success ? "advance_success" : "advance_failed",
                    droneState
                );

                if (!success) {
                    std::cout << "Advance failed; drone stayed in place."
                              << std::endl;
                }

                break;
            }

            case CommandType::Elevate: {
                bool success = movementDriver.elevate(command.distance);

                drone.receiveMovementResult(success);

                writeLogRow(
                    logFile,
                    iteration + 1,
                    success ? "elevate_success" : "elevate_failed",
                    droneState
                );

                if (!success) {
                    std::cout << "Elevate failed; drone stayed in place."
                              << std::endl;
                }

                break;
            }

            case CommandType::Finished: {
                writeLogRow(
                    logFile,
                    iteration + 1,
                    "finished",
                    droneState
                );

                std::cout << "Drone reported Finished" << std::endl;

                MapFileWriter::writeSparseMap(mapOutputPath, droneMap,missionConfig);

                std::cout << "Wrote output map to: "
                          << mapOutputPath
                          << std::endl;

                double score =
                    ScoreCalculator::calculateScore(worldMap, droneMap,missionConfig);

                std::cout << "Mapping score: "
                          << score
                          << "/100"
                          << std::endl;

                return 0;
            }
        }
    }

    std::cout << "Simulation stopped: maximum iteration limit reached."
              << std::endl;

    writeLogRow(
        logFile,
        maxIterations,
        "max_iterations_reached",
        droneState
    );

    MapFileWriter::writeSparseMap(mapOutputPath, droneMap, missionConfig);

    double score =
        ScoreCalculator::calculateScore(worldMap, droneMap,missionConfig);

    std::cout << "Partial mapping score: "
              << score
              << "/100"
              << std::endl;

    return 1;
}