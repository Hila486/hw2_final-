#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

// **Do not change this interface.**
class IGPS {
public:
    virtual ~IGPS() = default; // virtual destructor

    [[nodiscard]] virtual Position3D position() const = 0; // returns the position of the drone, const means the function does not modify the object
    [[nodiscard]] virtual Orientation heading() const = 0; // returns the heading of the drone, const means the function does not modify the object
};

} // namespace drone_mapper
