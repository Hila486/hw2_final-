#include <drone_mapper/MockMovement.h>

#include <cmath>
#include <mp-units/systems/si/math.h>

namespace drone_mapper {

namespace {

constexpr double kTrigEpsilon = 1.0e-9;

[[nodiscard]] double horizontalDegrees(HorizontalAngle angle) {
    return angle.force_numerical_value_in(deg);
}

[[nodiscard]] double normalizeDegrees(double degrees) {
    double normalized = std::fmod(degrees, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized;
}

[[nodiscard]] double snapped(double value) {
    return std::abs(value) < kTrigEpsilon ? 0.0 : value;
}

} // namespace

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

// Rotate the drone relative to its current heading
types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;

    const double next_degrees =
        normalizeDegrees(horizontalDegrees(current.horizontal + signed_angle));

    gps_.setHeading(Orientation{next_degrees * horizontal_angle[deg], current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D current_pos = gps_.position();
    const Orientation heading = gps_.heading();
    const double distance_cm = distance.force_numerical_value_in(cm);

    const double dx_cm =
        distance_cm * snapped(si::cos(heading.horizontal).force_numerical_value_in(mp::one));
    const double dy_cm =
        distance_cm * snapped(si::sin(heading.horizontal).force_numerical_value_in(mp::one));

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
