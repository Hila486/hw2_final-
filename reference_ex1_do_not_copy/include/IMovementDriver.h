#pragma once
// IMovementDriver.h
#include "Types.h"

// Interface for the drone movement driver.
// The movement driver is responsible for executing movement commands (rotate, advance, elevate) and updating the drone's pose accordingly.
class IMovementDriver {
public:
    //virtual destructor for proper cleanup of derived classes.
    virtual ~IMovementDriver() = default;

    // Delete copy constructor and assignment operator to prevent copying of the movement driver.
    IMovementDriver(const IMovementDriver&) = delete;
    IMovementDriver& operator=(const IMovementDriver&) = delete;

    // Executes rotation, advance, or elevation commands.
    virtual bool rotate(RotationDirection direction, Degree angle) = 0;
    virtual bool advance(Cm distance) = 0;
    virtual bool elevate(Cm distance) = 0;

protected:
// Protected default constructor to allow construction of derived classes, but prevent direct instantiation of the interface.
    IMovementDriver() = default;
};

//virtual - This function can be replaced/implemented by a child class, and when I call it through a parent pointer, C++ should use the child version.”