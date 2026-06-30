// ConfigParser.cpp
#include "ConfigParser.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace { //creates a private area in this file for helper functions and variables

    // List of input errors collected during parsing.
    std::vector<std::string> inputErrors;

    // Adds an input error message to the list.
    void addInputError(const std::string& message) {
        inputErrors.push_back(message);
    }

    // Tries to convert a string token to an integer, and adds an input error if it fails.
    bool parseIntToken(
        const std::string& token,
        int& value,
        const std::string& fieldName,
        const std::string& filePath
    ) {
        try {
            size_t usedChars = 0;
            int parsedValue = std::stoi(token, &usedChars);

            if (usedChars != token.size()) {
                addInputError(
                    filePath + ": bad value for " + fieldName +
                    " = '" + token + "'. Ignoring this token."
                );
                return false;
            }

            value = parsedValue;
            return true;
        } catch (...) {
            addInputError(
                filePath + ": bad value for " + fieldName +
                " = '" + token + "'. Ignoring this token."
            );
            return false;
        }
    }

    // Validates that a value is positive, and adds an input error if it is not.
     // If the value is invalid, it is set to the provided default value.
    void validatePositive(
        int& value,
        int defaultValue,
        const std::string& fieldName,
        const std::string& filePath
    ) {
        if (value <= 0) {
            addInputError(
                filePath + ": " + fieldName +
                " must be positive. Got " + std::to_string(value) +
                ". Using default value " + std::to_string(defaultValue) + "."
            );

            value = defaultValue;
        }
    }

    // Validates that a value is non-negative, and adds an input error if it is not.
     // If the value is invalid, it is set to the provided default value.
    void validateNonNegative(
        int& value,
        int defaultValue,
        const std::string& fieldName,
        const std::string& filePath
    ) {
        if (value < 0) {
            addInputError(
                filePath + ": " + fieldName +
                " cannot be negative. Got " + std::to_string(value) +
                ". Using default value " + std::to_string(defaultValue) + "."
            );

            value = defaultValue;
        }
    }

    // Validates that a resolution value is supported, and adds an input error if it is not.
    // If the value is invalid, it is set to the supported resolution.
    void validateSupportedResolution(
        int& resolutionCm,
        const std::string& filePath
    ) {
        if (resolutionCm <= 0) {
            addInputError(
                filePath + ": resolutionCm must be positive. Got " +
                std::to_string(resolutionCm) +
                ". Using supported resolutionCm = " +
                std::to_string(SUPPORTED_RESOLUTION_CM) + "."
            );

            resolutionCm = SUPPORTED_RESOLUTION_CM;
            return;
        }

        if (resolutionCm != SUPPORTED_RESOLUTION_CM) {
            addInputError(
                filePath + ": unsupported resolutionCm = " +
                std::to_string(resolutionCm) +
                ". This project supports only resolutionCm = " +
                std::to_string(SUPPORTED_RESOLUTION_CM) +
                ". Using supported value."
            );

            resolutionCm = SUPPORTED_RESOLUTION_CM;
        }
    }

    // Helper function to read an integer field from the file, with error handling.
    int readIntField(std::ifstream& file,
                 const std::string& filePath,
                 const std::string& fieldName,
                 int defaultValue) {
    std::string token;

    if (!(file >> token)) {
        addInputError(
            filePath + ": missing " + fieldName +
            ". Using default value " + std::to_string(defaultValue) + "."
        );
        return defaultValue;
    }

    int value = defaultValue;

    if (!parseIntToken(token, value, fieldName, filePath)) {
        addInputError(
            filePath + ": invalid " + fieldName +
            ". Using default value " + std::to_string(defaultValue) + "."
        );
        return defaultValue;
    }

    return value;
}
}

/*
    Responsible for reading the input files of the simulator:
    1. drone_config.txt   -> creates DroneConfig
    2. mission_config.txt -> creates MissionConfig
    3. map_input.txt      -> creates GroundTruthMap
*/

// Reads drone_config.txt and creates a DroneConfig object.
DroneConfig ConfigParser::parseDroneConfig(const std::string& filePath) {
    /*
        Expected drone_config.txt format:
        maxAdvanceCm
        maxElevateCm
        maxRotateDeg
        lidarMinRangeCm
        lidarMaxRangeCm
        lidarBeamSpacingCm
        lidarFovCircleCount
        minPassWidthCm
        minPassLengthCm
        minPassHeightCm
    */

    DroneConfig config;

    std::ifstream file(filePath);

    // If the file cannot be opened, we add an input error and return a default config.
    if (!file.is_open()) {
        addInputError(
            filePath +
            ": could not open file. Using default drone configuration."
        );

        config.maxAdvance = 100 * cm;
        config.maxElevate = 50 * cm;
        config.maxRotate = 90 * deg;

        config.lidarMinRange = 0 * cm;
        config.lidarMaxRange = 500 * cm;
        config.lidarBeamSpacing = 1 * cm;
        config.lidarFovCircleCount = 1;

        config.minPassageWidth = 1 * cm;
        config.minPassageLength = 1 * cm;
        config.minPassageHeight = 1 * cm;

        return config;
    }

    //else we read the file and parse the values, while collecting input errors for any issues we find:


    // Read all fields from the file, with error handling.
    int maxAdvanceCm = readIntField(file, filePath, "maxAdvanceCm", 100);
    int maxElevateCm = readIntField(file, filePath, "maxElevateCm", 50);
    int maxRotateDeg = readIntField(file, filePath, "maxRotateDeg", 90);

    int lidarMinRangeCm = readIntField(file, filePath, "lidarMinRangeCm", 0);
    int lidarMaxRangeCm = readIntField(file, filePath, "lidarMaxRangeCm", 500);
    int lidarBeamSpacingCm = readIntField(file, filePath, "lidarBeamSpacingCm", 1);
    int lidarFovCircleCount = readIntField(file, filePath, "lidarFovCircleCount", 1);

    int minPassWidthCm = readIntField(file, filePath, "minPassWidthCm", 1);
    int minPassLengthCm = readIntField(file, filePath, "minPassLengthCm", 1);
    int minPassHeightCm = readIntField(file, filePath, "minPassHeightCm", 1);

    validatePositive(maxAdvanceCm, 100, "maxAdvanceCm", filePath);
    validatePositive(maxElevateCm, 50, "maxElevateCm", filePath);
    validatePositive(maxRotateDeg, 90, "maxRotateDeg", filePath);

    validateNonNegative(lidarMinRangeCm, 0, "lidarMinRangeCm", filePath);
    validatePositive(lidarMaxRangeCm, 500, "lidarMaxRangeCm", filePath);
    validatePositive(lidarBeamSpacingCm, 1, "lidarBeamSpacingCm", filePath);
    validatePositive(lidarFovCircleCount, 1, "lidarFovCircleCount", filePath);

    validatePositive(minPassWidthCm, 1, "minPassWidthCm", filePath);
    validatePositive(minPassLengthCm, 1, "minPassLengthCm", filePath);
    validatePositive(minPassHeightCm, 1, "minPassHeightCm", filePath);

    
    if (lidarMinRangeCm > lidarMaxRangeCm) {
        addInputError(
            filePath +
            ": lidarMinRangeCm is larger than lidarMaxRangeCm. "
            "Using lidarMinRangeCm = 0."
        );

        lidarMinRangeCm = 0;
    }
    // After reading and validating all fields, we set the config values.
    config.maxAdvance = maxAdvanceCm * cm;
    config.maxElevate = maxElevateCm * cm;
    config.maxRotate = maxRotateDeg * deg;

    config.lidarMinRange = lidarMinRangeCm * cm;
    config.lidarMaxRange = lidarMaxRangeCm * cm;
    config.lidarBeamSpacing = lidarBeamSpacingCm * cm;
    config.lidarFovCircleCount = lidarFovCircleCount;

    config.minPassageWidth = minPassWidthCm * cm;
    config.minPassageLength = minPassLengthCm * cm;
    config.minPassageHeight = minPassHeightCm * cm;

    // Finally, we return the config object.
    return config;
}
// Reads mission_config.txt and creates a MissionConfig object.
MissionConfig ConfigParser::parseMissionConfig(const std::string& filePath) {
    /*
        Expected mission_config.txt format:

        startX startY startHeight
        startAngleDeg
        minX maxX
        minY maxY
        minZ maxZ
        resolutionCm
    */

    MissionConfig config;

    std::ifstream file(filePath);

    // If the file cannot be opened, we add an input error and return a default config.
    if (!file.is_open()) {
        addInputError(
            filePath +
            ": could not open file. Using default mission configuration."
        );

        config.startPosition = Position{0 * cm, 0 * cm, 0 * cm};
        config.startAngleDeg = 0 * deg;

        config.minX = 0 * cm;
        config.maxX = 10 * cm;

        config.minY = 0 * cm;
        config.maxY = 10 * cm;

        config.minZ = 0 * cm;
        config.maxZ = 3 * cm;

        config.resolutionCm = SUPPORTED_RESOLUTION_CM * cm;

        return config;
    }

    //else we read the file and parse the values, while collecting input errors for any issues we find:

    int startX = readIntField(file, filePath, "startX", 0);
    int startY = readIntField(file, filePath, "startY", 0);
    int startHeight = readIntField(file, filePath, "startHeight", 0);

    int startAngle = readIntField(file, filePath, "startAngleDeg", 0);

    int minX = readIntField(file, filePath, "minX", 0);
    int maxX = readIntField(file, filePath, "maxX", 10);

    int minY = readIntField(file, filePath, "minY", 0);
    int maxY = readIntField(file, filePath, "maxY", 10);

    int minZ = readIntField(file, filePath, "minZ", 0);
    int maxZ = readIntField(file, filePath, "maxZ", 3);

    int resolution = readIntField(file, filePath, "resolutionCm", SUPPORTED_RESOLUTION_CM);


    // Validate the fields and collect input errors for any issues found.
    if (startAngle < 0 || startAngle >= 360) {
        addInputError(
            filePath + ": startAngleDeg should be in range [0, 359]. Got " +
            std::to_string(startAngle) + ". Normalizing it."
        );

        startAngle = startAngle % 360;

        if (startAngle < 0) {
            startAngle += 360;
        }
    }

    if (minX > maxX) {
        addInputError(
            filePath + ": minX is larger than maxX. Swapping them."
        );

        int temp = minX;
        minX = maxX;
        maxX = temp;
    }

    if (minY > maxY) {
        addInputError(
            filePath + ": minY is larger than maxY. Swapping them."
        );

        int temp = minY;
        minY = maxY;
        maxY = temp;
    }

    if (minZ > maxZ) {
        addInputError(
            filePath + ": minZ is larger than maxZ. Swapping them."
        );

        int temp = minZ;
        minZ = maxZ;
        maxZ = temp;
    }
    
    validateSupportedResolution(resolution, filePath);

    // After validating all fields, we set the config values.
    config.startPosition = Position{
        startX * cm,
        startY * cm,
        startHeight * cm
    };

    config.startAngleDeg = startAngle * deg;

    config.minX = minX * cm;
    config.maxX = maxX * cm;

    config.minY = minY * cm;
    config.maxY = maxY * cm;

    config.minZ = minZ * cm;
    config.maxZ = maxZ * cm;

    config.resolutionCm = resolution * cm;

    bool startOutside =
        config.startPosition.x < config.minX ||
        config.startPosition.x > config.maxX ||
        config.startPosition.y < config.minY ||
        config.startPosition.y > config.maxY ||
        config.startPosition.height < config.minZ ||
        config.startPosition.height > config.maxZ;


    if (startOutside) {
        addInputError(
            filePath +
            ": start position is outside mission boundaries. "
            "Using minimum boundary position as start position."
        );

        config.startPosition = Position{
            config.minX,
            config.minY,
            config.minZ
        };
    }
    // Finally, we return the config object.
    return config;
}

// Reads map_input.txt and creates a GroundTruthMap object.
GroundTruthMap ConfigParser::parseMapInput(const std::string& filePath) {
    /*
        Expected map_input.txt format:

        sizeX sizeY sizeHeight

        Then sizeX * sizeY * sizeHeight cell values.

        Cell values:
        0 = free
        1 = occupied
    */

    std::ifstream file(filePath);

    // If the file cannot be opened, we add an input error and return a default map.
    if (!file.is_open()) {
        addInputError(
            filePath +
            ": could not open file. Using default empty map 10x10x3."
        );

        return GroundTruthMap(10, 10, 3);
    }

    //else we read the file and parse the values, while collecting input errors for any issues we find:

    int sizeX = readIntField(file, filePath, "sizeX", 10);
    int sizeY = readIntField(file, filePath, "sizeY", 10);
    int sizeHeight = readIntField(file, filePath, "sizeHeight", 3);

    validatePositive(sizeX, 10, "sizeX", filePath);
    validatePositive(sizeY, 10, "sizeY", filePath);
    validatePositive(sizeHeight, 3, "sizeHeight", filePath);

    // After validating the fields, we create the map and read the cell values.
    GroundTruthMap map(sizeX, sizeY, sizeHeight);

    int expectedCells = sizeX * sizeY * sizeHeight;

    // We read the cell values in the order of height, then y, then x.
    for (int height = 0; height < sizeHeight; ++height) {
        for (int y = 0; y < sizeY; ++y) {
            for (int x = 0; x < sizeX; ++x) {
                std::string token;

                // If we cannot read a token for the cell value, we add an input error and use 0 = free for this cell.
                if (!(file >> token)) {
                    addInputError(
                        filePath + ": missing cell value at position (" +
                        std::to_string(x) + ", " +
                        std::to_string(y) + ", " +
                        std::to_string(height) +
                        "). Using 0 = free."
                    );

                    map.setCell(
                        Position{x * cm, y * cm, height * cm},
                        CellState::Free
                    );
                    continue;
                }
                //else we try to parse the token as an integer cell value, and add an input error if it is invalid. 

                int value = 0;
                if (!parseIntToken(token, value, "map cell", filePath)) {
                    addInputError(
                        filePath + ": using 0 = free for bad cell at position (" +
                        std::to_string(x) + ", " +
                        std::to_string(y) + ", " +
                        std::to_string(height) + ")."
                    );

                    map.setCell(
                        Position{x * cm, y * cm, height * cm},
                        CellState::Free
                    );
                    continue;
                }

                if (value == 0) {
                    map.setCell(
                        Position{x * cm, y * cm, height * cm},
                        CellState::Free
                    );
                } else if (value == 1) {
                    map.setCell(
                        Position{x * cm, y * cm, height * cm},
                        CellState::Occupied
                    );
                } else {
                    addInputError(
                        filePath + ": invalid cell value " +
                        std::to_string(value) +
                        " at position (" +
                        std::to_string(x) + ", " +
                        std::to_string(y) + ", " +
                        std::to_string(height) +
                        "). Expected 0 or 1. Using 0 = free."
                    );

                    map.setCell(
                        Position{x * cm, y * cm, height * cm},
                        CellState::Free
                    );
                }
            }
        }
    }

    std::string extraToken;
    int extraCount = 0;

    // After reading the expected number of cell values, we check if there are extra tokens in the file, and add an input error if we find any.
    while (file >> extraToken) {
        ++extraCount;
    }

    if (extraCount > 0) {
        addInputError(
            filePath + ": found " +
            std::to_string(extraCount) +
            " extra cell value(s) after expected " +
            std::to_string(expectedCells) +
            " cells. Ignoring extras."
        );
    }

    return map;
}
// Clears the list of collected input errors.
void ConfigParser::clearInputErrors() {
    inputErrors.clear();
}
// Writes the collected input errors to a file, one per line. If there are no errors, removes any existing file at the path.
void ConfigParser::writeInputErrors(const std::string& filePath) {
    /*
        Assignment requirement:
        Create input_errors.txt only if there are recoverable input errors.

        So if there are no errors, we remove an old file if it exists.
    */

    if (inputErrors.empty()) {
        std::remove(filePath.c_str());
        return;
    }

    std::ofstream file(filePath);

    if (!file.is_open()) {
        std::cout << "Could not create input errors file: "
                  << filePath
                  << std::endl;
        return;
    }

    for (const std::string& error : inputErrors) {
        file << error << "\n";
    }
}    