// Types.h
#pragma once


#include <vector>
#include "Units.h"

// --------------------
// Position / Pose
// --------------------


// Position in 3D space, with x, y, and height components.
struct Position {
    Cm x;
    Cm y;
    Cm height;

    //base constructor initializes to (0, 0, 0)
    Position() : x(0 * cm), y(0 * cm), height(0 * cm) {}

    //constructor with parameters
    Position(Cm x_, Cm y_, Cm height_)
        : x(x_), y(y_), height(height_) {}
};


//Full drone state: where the drone is + which direction it faces
// convention:
// 0   = east
// 90  = south
// 180 = west
// 270 = north
struct Pose {
    Position position;
    Degree xyAngle;

    //base constructor initializes to (0, 0, 0) and 0 degrees
    Pose() : position(), xyAngle(0 * deg) {}

    //constructor with parameters
    Pose(const Position& position_, Degree xyAngle_)
        : position(position_), xyAngle(xyAngle_) {}
};

// --------------------
// Map cells
// --------------------
enum class CellState : int {
    Unknown = UNKNOWN_CELL,
    Free = FREE_CELL,
    Occupied = OCCUPIED_CELL,
    OutOfBounds = OUT_OF_BOUNDS
};

// Commands:

enum class RotationDirection {
    Left,
    Right
};

enum class CommandType {
    Rotate,
    Advance,
    Elevate,
    Scan,
    GetLocation,
    Finished
};
// Scan angle: horizontal (xy) and vertical (height)
struct ScanAngle {
    Degree xyAngle;
    Degree heightAngle;

    //base constructor initializes to 0 degrees for both angles
    ScanAngle() : xyAngle(0 * deg), heightAngle(0 * deg) {}

    //constructor with parameters
    ScanAngle(Degree xyAngle_, Degree heightAngle_)
        : xyAngle(xyAngle_), heightAngle(heightAngle_) {}
};

// Command struct: represents a command sent from the drone to the simulator.
struct Command {
    CommandType type;

    // For Rotate command, specifies the direction (left or right) and angle of rotation.
    RotationDirection rotationDirection;
    Degree angle;

    // For Advance and Elevate commands, specifies the distance to move.
    Cm distance;

    // For Scan command, specifies the angles to scan.
    ScanAngle scanAngle;

    // Default constructor initializes to a Finished command (no action).
    Command()
        : type(CommandType::Finished),
          rotationDirection(RotationDirection::Right),
          angle(0 * deg),
          distance(0 * cm),
          scanAngle() {}


    // Static factory methods for creating specific commands:
    static Command rotate(RotationDirection dir, Degree angle_) {
        Command command;
        command.type = CommandType::Rotate;
        command.rotationDirection = dir;
        command.angle = angle_;
        return command;
    }
    
    static Command advance(Cm distance_) {
        Command command;
        command.type = CommandType::Advance;
        command.distance = distance_;
        return command;
    }

    static Command elevate(Cm distance_) {
        Command command;
        command.type = CommandType::Elevate;
        command.distance = distance_;
        return command;
    }

    static Command scan(ScanAngle angle_) {
        Command command;
        command.type = CommandType::Scan;
        command.scanAngle = angle_;
        return command;
    }

    static Command getLocation() {
        Command command;
        command.type = CommandType::GetLocation;
        return command;
    }

    static Command finished() {
        Command command;
        command.type = CommandType::Finished;
        return command;
    }
};

// --------------------
// Lidar result
// --------------------

// Represents represents one lidar beam result
struct ScanHit {

    //stores: the angle of the beam, the distance where something was detected, whether the beam actually hit an object.
    ScanAngle angle;
    Cm distance;
    bool hitObject;

    //base constructor initializes to 0 degrees, 0 cm, and no hit
    ScanHit()
        : angle(),
          distance(0 * cm),
          hitObject(false) {
    }

    //constructor with parameters
    ScanHit(const ScanAngle& angle_, Cm distance_, bool hitObject_)
        : angle(angle_),
          distance(distance_),
          hitObject(hitObject_) {
    }
};
// A ScanResult is a collection of ScanHits.
using ScanResult = std::vector<ScanHit>;
