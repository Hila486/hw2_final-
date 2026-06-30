#include <drone_mapper/MockMovement.h>

#include <mp-units/systems/si/math.h>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

// Rotate the drone relative to its current heading
types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading(); // get the orientation of the drone, horizontal and altitude angles
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D current_pos = gps_.position();
    const Orientation heading = gps_.heading();
    const double distance_cm = distance.force_numerical_value_in(cm);
    const double dx_cm = distance_cm * si::cos(heading.horizontal).force_numerical_value_in(mp::one);
    const double dy_cm = distance_cm * si::sin(heading.horizontal).force_numerical_value_in(mp::one);

    gps_.setPosition(Position3D{
        current_pos.x + dx_cm * x_extent[cm],
        current_pos.y + dy_cm * y_extent[cm],
        current_pos.z,
    });
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D current_pos = gps_.position();
    const double distance_cm = distance.force_numerical_value_in(cm);
    gps_.setPosition(Position3D{
        current_pos.x,
        current_pos.y,
        current_pos.z + distance_cm * z_extent[cm],
    });
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
