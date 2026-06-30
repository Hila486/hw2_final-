//MockLidarSensor.cpp
#include "MockLidarSensor.h"

#include <cmath>
#include <set>
#include <utility>


namespace {
    // Helper functions to convert units and perform common calculations.
    constexpr double PI = 3.14159265358979323846;

    double radiansToDegrees(double radians) {
        return radians * 180.0 / PI;
    }

    int cmToInt(Cm value) {
        return static_cast<int>(value.numerical_value_in(cm));
    }

    double cmToDouble(Cm value) {
        return static_cast<double>(value.numerical_value_in(cm));
    }

    int degToInt(Degree value) {
        return static_cast<int>(value.numerical_value_in(deg));
    }
}

// Constructor.
MockLidarSensor::MockLidarSensor(
    const DroneConfig& droneConfig,
    const GroundTruthMap& worldMap,
    const IPositionSensor& positionSensor
)
    : droneConfig(droneConfig),
      worldMap(worldMap),
      positionSensor(positionSensor) {
}

// Normalizes an angle to the range [0, 360) degrees.
Degree MockLidarSensor::normalizeAngle(Degree angle) const {
    int result = degToInt(angle) % 360;

    if (result < 0) {
        result += 360;
    }

    return result * deg;
}

// Performs a lidar scan at the given scan angle, and returns a list of scan hits.
ScanResult MockLidarSensor::scan(const ScanAngle& scanAngle) const {
    ScanResult result;

    Pose currentPose = positionSensor.getPose();

    int circleCount = droneConfig.lidarFovCircleCount;

    if (circleCount <= 0) {
        circleCount = 1;
    }

    int spacingCm = cmToInt(droneConfig.lidarBeamSpacing);

    if (spacingCm <= 0) {
        spacingCm = 1;
    }

    int zMin = cmToInt(droneConfig.lidarMinRange);

    if (zMin <= 0) {
        zMin = 1;
    }

    // We store angles as plain integers in the set.
    // This is safer than storing Degree directly, because std::set needs comparison.
    std::set<std::pair<int, int>> usedRelativeAngles;


    // We start with the central beam, which has the exact requested angles.
    usedRelativeAngles.insert({
        degToInt(scanAngle.xyAngle),
        degToInt(scanAngle.heightAngle)
    });
    
    Degree absoluteCenterXyAngle =
        normalizeAngle(currentPose.xyAngle + scanAngle.xyAngle);

    Cm centerDistance = 0 * cm;

    bool centerHit = traceBeam(
        absoluteCenterXyAngle,
        scanAngle.heightAngle,
        centerDistance
    );

    // The central beam is the first in the result, even if it did not hit anything.
    result.push_back(
        ScanHit(scanAngle, centerDistance, centerHit)
    );

    // now we add more beams in concentric circles around the central beam, until we reach the requested number of circles.
    int beamsInCircle = 1;
    // Each new circle has 4 times more beams than the previous one, because we double the resolution in both angles.
    for (int circleIndex = 1; circleIndex < circleCount; ++circleIndex) {
        beamsInCircle *= 4;

        // calculate the angle offset for this circle, based on the desired spacing in cm at Z-min.
        double radiusAtZMin =
            static_cast<double>(circleIndex * spacingCm);

        // angle = atan(radius / z) => angle = atan(radius * 1/z) => angle = atan(radius * 1/Z-min)
        double angleOffsetDegrees =
            radiansToDegrees(std::atan(radiusAtZMin / static_cast<double>(zMin)));

        // We want to distribute the beams evenly in the circle, so we calculate their angles based on their index and the total number of beams in this circle.
        for (int beamIndex = 0; beamIndex < beamsInCircle; ++beamIndex) {
            
            double phase =
                2.0 * PI * static_cast<double>(beamIndex) /
                static_cast<double>(beamsInCircle);

            
            Degree relativeXyOffset =
                static_cast<int>(
                    std::round(angleOffsetDegrees * std::cos(phase))
                ) * deg;

            Degree relativeHeightOffset =
                static_cast<int>(
                    std::round(angleOffsetDegrees * std::sin(phase))
                ) * deg;

            ScanAngle beamRelativeAngle{
                scanAngle.xyAngle + relativeXyOffset,
                scanAngle.heightAngle + relativeHeightOffset
            };

            auto key = std::make_pair(
                degToInt(beamRelativeAngle.xyAngle),
                degToInt(beamRelativeAngle.heightAngle)
            );

            // If we already traced a beam with this relative angle, skip it.
            if (usedRelativeAngles.find(key) != usedRelativeAngles.end()) {
                continue;
            }

            usedRelativeAngles.insert(key);

            Degree beamAbsoluteXyAngle =
                normalizeAngle(currentPose.xyAngle + beamRelativeAngle.xyAngle);

            Cm beamDistance = 0 * cm;

            bool beamHit = traceBeam(
                beamAbsoluteXyAngle,
                beamRelativeAngle.heightAngle,
                beamDistance
            );
            
            result.push_back(
                ScanHit(beamRelativeAngle, beamDistance, beamHit)
            );
        }
    }

    return result;
}

// Traces a single lidar beam from the drone's current position at the specified angles.
bool MockLidarSensor::traceBeam(
    Degree absoluteXyAngle,
    Degree heightAngle,
    Cm& beamDistance
) const {
    Pose currentPose = positionSensor.getPose();
    Position origin = currentPose.position;

    int maxRangeCm = cmToInt(droneConfig.lidarMaxRange);

    if (maxRangeCm <= 0) {
        beamDistance = 0 * cm;
        return false;
    }

    int minRangeCm = cmToInt(droneConfig.lidarMinRange);

    double xyRadians =
        static_cast<double>(degToInt(absoluteXyAngle)) * PI / 180.0;

    double heightRadians =
        static_cast<double>(degToInt(heightAngle)) * PI / 180.0;

    double horizontalFactor = std::cos(heightRadians);

    double dx = horizontalFactor * std::cos(xyRadians);
    double dy = horizontalFactor * std::sin(xyRadians);
    double dh = std::sin(heightRadians);

    int lastClearDistanceCm = 0;

    // We check points along the beam at the supported resolution intervals, until we reach the max range.
    for (int distanceCm = SUPPORTED_RESOLUTION_CM;
         distanceCm <= maxRangeCm;
         distanceCm += SUPPORTED_RESOLUTION_CM) {

        double rawX =
            cmToDouble(origin.x) + dx * static_cast<double>(distanceCm);

        double rawY =
            cmToDouble(origin.y) + dy * static_cast<double>(distanceCm);

        double rawH =
            cmToDouble(origin.height) + dh * static_cast<double>(distanceCm);

        Position sample{
            static_cast<int>(std::round(rawX)) * cm,
            static_cast<int>(std::round(rawY)) * cm,
            static_cast<int>(std::round(rawH)) * cm
        };

        // If the sample point is the same as the origin, skip it, because we assume the drone's own cell is always free and we want to find the first hit after it.
        if (sample.x == origin.x &&
            sample.y == origin.y &&
            sample.height == origin.height) {
            continue;
        }

        CellState cell = worldMap.getCell(sample);

        // If the beam leaves the map, stop tracing.
        // This is not considered an object hit.
        // We return the last confirmed clear distance.
        if (cell == CellState::OutOfBounds) {
            beamDistance = lastClearDistanceCm * cm;
            return false;
        }

        // If the beam reaches an occupied cell, this is a lidar hit.
        if (cell == CellState::Occupied) {
            if (minRangeCm > 0 && distanceCm < minRangeCm) {
                /*
                    The beam detected something too close to measure accurately.
                    HW meaning: distance 0 means "hit too close".
                */
                beamDistance = 0 * cm;
            } else {
                beamDistance = distanceCm * cm;
            }

            return true;
        }

        // The beam is clear at this point, we can continue. We also update the last clear distance, so if we later hit something or go out of bounds, we know the exact distance to the last clear point.
        lastClearDistanceCm = distanceCm;
    }

    // We reached the max range without hitting anything, so we return the max range as the distance and false for no hit.
    beamDistance = maxRangeCm * cm;
    return false;
}