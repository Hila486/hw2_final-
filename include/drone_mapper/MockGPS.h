#pragma once

#include <drone_mapper/IGPS.h>

namespace drone_mapper {

class MockGPS final : public IGPS { // mock GPS class that implements the IGPS interface
public:
    MockGPS(Position3D position, Orientation heading, PhysicalLength resolution);

    [[nodiscard]] Position3D position() const override; // returns the position of the drone, const means the function does not modify the object
    [[nodiscard]] Orientation heading() const override; // returns the heading of the drone, const means the function does not modify the object

    void setPosition(Position3D position);// sets the position of the drone
    void setHeading(Orientation heading);// sets the heading of the drone

private:
    Position3D position_{};
    Orientation heading_{};
    PhysicalLength resolution_{};
};

} // namespace drone_mapper
