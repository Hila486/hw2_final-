#pragma once
// ConfigParser.h
#include <string>

#include "DroneConfig.h"
#include "MissionConfig.h"
#include "GroundTruthMap.h"

/*
    Reads the simulator input files and converts them into C++ objects:
    - drone_config.txt   -> DroneConfig
    - mission_config.txt -> MissionConfig
    - map_input.txt      -> GroundTruthMap

    It also collects recoverable input errors.
    At the end, Simulator writes them to input_errors.txt.
*/
class ConfigParser {
public:
    //Clears the list of remembered input errors
    static void clearInputErrors();

    // Writes the collected input errors to a file.
    static void writeInputErrors(const std::string& filePath);

    // Parses the drone configuration file and returns a DroneConfig object.
    static DroneConfig parseDroneConfig(const std::string& filePath);

    // Parses the mission configuration file and returns a MissionConfig object.
    static MissionConfig parseMissionConfig(const std::string& filePath);

    // Parses the map input file and returns a GroundTruthMap object.
    static GroundTruthMap parseMapInput(const std::string& filePath);
};