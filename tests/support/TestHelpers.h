#pragma once

// Shared helpers and GMock doubles for the component and integration tests.
// Everything lives in drone_mapper::test to keep test translation units tidy.

#include <TinyNPY.h>

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/IMutableMap3D.h>
#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/NpyMapIO.h>
#include <drone_mapper/Types.h>
#include <drone_mapper/Units.h>

#include <gmock/gmock.h>

#include <atomic>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace drone_mapper::test {

// ---------------------------------------------------------------------------
// Strong-type construction helpers (mp-units quantities are verbose to build).
// ---------------------------------------------------------------------------

[[nodiscard]] inline XLength X(double value) { return value * x_extent[cm]; }
[[nodiscard]] inline YLength Y(double value) { return value * y_extent[cm]; }
[[nodiscard]] inline ZLength Z(double value) { return value * z_extent[cm]; }
[[nodiscard]] inline PhysicalLength L(double value) { return value * cm; }
[[nodiscard]] inline HorizontalAngle Hdeg(double value) { return value * horizontal_angle[deg]; }
[[nodiscard]] inline AltitudeAngle Adeg(double value) { return value * altitude_angle[deg]; }

[[nodiscard]] inline Position3D P(double x_cm, double y_cm, double z_cm) {
    return Position3D{X(x_cm), Y(y_cm), Z(z_cm)};
}

[[nodiscard]] inline Orientation heading(double horizontal_deg, double altitude_deg = 0.0) {
    return Orientation{Hdeg(horizontal_deg), Adeg(altitude_deg)};
}

// ---------------------------------------------------------------------------
// NPY map builders. The NPY layout is row-major (depth, height, width) = (z, y, x).
// ---------------------------------------------------------------------------

[[nodiscard]] inline std::shared_ptr<NpyArray>
makeMapArray(std::size_t depth, std::size_t height, std::size_t width, int fill_value = 0) {
    return makeFilledIntNpyArray(NpyMapShape{depth, height, width}, fill_value);
}

// Writes a single voxel using the same (z, y, x) flattening Map3DImpl expects.
inline void setVoxelRaw(NpyArray& array,
                        std::size_t height,
                        std::size_t width,
                        std::size_t z,
                        std::size_t y,
                        std::size_t x,
                        int value) {
    array.Data<int>()[z * (height * width) + y * width + x] = value;
}

// A full-extent map config: offset (0,0,0) and boundaries spanning the whole
// array, i.e. x in [0, width*res], y in [0, height*res], z in [0, depth*res].
[[nodiscard]] inline types::MapConfig
fullConfig(std::size_t depth, std::size_t height, std::size_t width, double resolution_cm) {
    return types::MapConfig{
        types::MappingBounds{
            X(0.0),
            X(static_cast<double>(width) * resolution_cm),
            Y(0.0),
            Y(static_cast<double>(height) * resolution_cm),
            Z(0.0),
            Z(static_cast<double>(depth) * resolution_cm),
        },
        Position3D{},
        L(resolution_cm),
    };
}

// ---------------------------------------------------------------------------
// Config builders with reasonable defaults the tests can tweak.
// ---------------------------------------------------------------------------

[[nodiscard]] inline types::DroneConfigData
makeDrone(double radius_cm = 5.0,
          double max_rotate_deg = 90.0,
          double max_advance_cm = 50.0,
          double max_elevate_cm = 40.0) {
    return types::DroneConfigData{
        L(radius_cm),
        Hdeg(max_rotate_deg),
        L(max_advance_cm),
        L(max_elevate_cm),
    };
}

[[nodiscard]] inline types::LidarConfigData
makeLidar(double z_min_cm = 20.0,
          double z_max_cm = 120.0,
          double d_cm = 2.5,
          std::size_t fov_circles = 1) {
    return types::LidarConfigData{L(z_min_cm), L(z_max_cm), L(d_cm), fov_circles};
}

[[nodiscard]] inline types::MissionConfigData
makeMission(std::size_t max_steps = 50,
            double gps_resolution_cm = 10.0,
            double output_mapping_resolution_factor = 1.0) {
    types::MissionConfigData mission;
    mission.max_steps = max_steps;
    mission.gps_resolution = L(gps_resolution_cm);
    mission.output_mapping_resolution_factor = output_mapping_resolution_factor;
    mission.mission_bounds = types::MappingBounds{};
    return mission;
}

// ---------------------------------------------------------------------------
// Temp directory helper (per-test isolated output location).
// ---------------------------------------------------------------------------

[[nodiscard]] inline std::filesystem::path makeTempDir(const std::string& label) {
    static std::atomic<unsigned long long> counter{0};
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() /
        ("drone_mapper_test_" + label + "_" + std::to_string(counter.fetch_add(1)));
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

// ---------------------------------------------------------------------------
// GMock doubles for the public interfaces.
// ---------------------------------------------------------------------------

class MockMappingAlgorithm : public IMappingAlgorithm {
public:
    MockMappingAlgorithm(const types::MissionConfigData& mission_config,
                         const types::LidarConfigData& lidar_config,
                         const types::DroneConfigData& drone_config,
                         const IMap3D& output_map)
        : IMappingAlgorithm(mission_config, lidar_config, drone_config, output_map) {}

    MOCK_METHOD(types::MappingStepCommand,
                nextStep,
                (const types::DroneState& state, const types::LidarScanResult* latest_scan),
                (override));
};

class MockDroneControl : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
    MOCK_METHOD(types::DroneState, state, (), (const, override));
};

class MockMissionControl : public IMissionControl {
public:
    MOCK_METHOD(types::MissionRunResult, runMission, (), (override));
};

class MockGpsInterface : public IGPS {
public:
    MOCK_METHOD(Position3D, position, (), (const, override));
    MOCK_METHOD(Orientation, heading, (), (const, override));
};

class MockMovementInterface : public IDroneMovement {
public:
    MOCK_METHOD(types::MovementResult, rotate,
                (types::RotationDirection direction, HorizontalAngle angle), (override));
    MOCK_METHOD(types::MovementResult, advance, (PhysicalLength distance), (override));
    MOCK_METHOD(types::MovementResult, elevate, (PhysicalLength distance), (override));
};

class MockLidarInterface : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation scan_orientation), (const, override));
    MOCK_METHOD(types::LidarConfigData, config, (), (const, override));
};

class MockSimulationRun : public ISimulationRun {
public:
    MOCK_METHOD(types::SimulationResult, run, (), (override));
};

class MockSimulationRunFactory : public ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<ISimulationRun>,
                create,
                (const types::SimulationConfigData& simulation,
                 const types::MissionConfigData& mission,
                 const types::DroneConfigData& drone,
                 const types::LidarConfigData& lidar,
                 const std::filesystem::path& output_path),
                (override));
};

} // namespace drone_mapper::test
