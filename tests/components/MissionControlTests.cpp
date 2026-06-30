#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>

#include "support/TestHelpers.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

namespace drone_mapper {
namespace {

using ::testing::NiceMock;
using ::testing::Return;

using test::fullConfig;
using test::heading;
using test::makeDrone;
using test::makeMapArray;
using test::makeMission;
using test::P;
using test::setVoxelRaw;

constexpr std::size_t kN = 10; // 10x10x10 map, res 10 -> world bounds [0,100]^3
constexpr double kRes = 10.0;

[[nodiscard]] types::DroneState stateAt(double x, double y, double z) {
    return types::DroneState{P(x, y, z), heading(0), 0};
}

[[nodiscard]] types::DroneStepResult stepContinue() {
    return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
}
[[nodiscard]] types::DroneStepResult stepCompleted() {
    return types::DroneStepResult{types::DroneStepStatus::Completed, {}};
}
[[nodiscard]] types::DroneStepResult stepError(const std::string& message) {
    return types::DroneStepResult{types::DroneStepStatus::Error, message};
}

struct Maps {
    std::unique_ptr<Map3DImpl> hidden;
    std::unique_ptr<Map3DImpl> output;
};

[[nodiscard]] Maps makeMaps(bool occupied_center) {
    const types::MapConfig config = fullConfig(kN, kN, kN, kRes);
    auto hidden_array = makeMapArray(kN, kN, kN, 0);
    if (occupied_center) {
        setVoxelRaw(*hidden_array, kN, kN, /*z=*/5, /*y=*/5, /*x=*/5, 1); // world center (55,55,55)
    }
    Maps maps;
    maps.hidden = std::make_unique<Map3DImpl>(std::move(hidden_array), config);
    maps.output = std::make_unique<Map3DImpl>(
        makeMapArray(kN, kN, kN, static_cast<int>(types::VoxelOccupancy::Unmapped)), config);
    return maps;
}

TEST(MissionControl, PreflightPassesAndCompletedMissionSavesMap) {
    Maps maps = makeMaps(/*occupied_center=*/false);
    NiceMock<test::MockDroneControl> drone_control;
    EXPECT_CALL(drone_control, state()).WillRepeatedly(Return(stateAt(50, 50, 50)));
    EXPECT_CALL(drone_control, step()).WillOnce(Return(stepCompleted()));

    const auto dir = test::makeTempDir("mc_completed");
    const std::filesystem::path output_file = dir / "run" / "output_map.npy";
    MissionControlImpl mission_control(makeMission(/*max_steps=*/10), makeDrone(/*radius=*/5),
                                       *maps.hidden, *maps.output, drone_control, output_file);

    const types::MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1u);
    EXPECT_TRUE(result.errors.empty());
    EXPECT_TRUE(std::filesystem::exists(output_file));
}

TEST(MissionControl, ReachesMaxStepsWhenDroneNeverCompletes) {
    Maps maps = makeMaps(false);
    NiceMock<test::MockDroneControl> drone_control;
    EXPECT_CALL(drone_control, state()).WillRepeatedly(Return(stateAt(50, 50, 50)));
    EXPECT_CALL(drone_control, step()).Times(3).WillRepeatedly(Return(stepContinue()));

    const auto dir = test::makeTempDir("mc_maxsteps");
    MissionControlImpl mission_control(makeMission(/*max_steps=*/3), makeDrone(5), *maps.hidden,
                                       *maps.output, drone_control, dir / "run" / "output_map.npy");

    const types::MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 3u);
}

TEST(MissionControl, PreflightOutOfBoundsErrorsWithoutStepping) {
    Maps maps = makeMaps(false);
    NiceMock<test::MockDroneControl> drone_control;
    // Sphere radius 5 at x=2 pokes to x=-3, outside the [0,100] map.
    EXPECT_CALL(drone_control, state()).WillRepeatedly(Return(stateAt(2, 50, 50)));
    EXPECT_CALL(drone_control, step()).Times(0);

    const auto dir = test::makeTempDir("mc_oob");
    const std::filesystem::path output_file = dir / "run" / "output_map.npy";
    MissionControlImpl mission_control(makeMission(10), makeDrone(5), *maps.hidden, *maps.output,
                                       drone_control, output_file);

    const types::MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 0u);
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors.front().code, "INITIAL_DRONE_OUT_OF_MAP");
    EXPECT_TRUE(std::filesystem::exists(output_file.parent_path() / "error.log"));
}

TEST(MissionControl, PreflightCollisionErrors) {
    Maps maps = makeMaps(/*occupied_center=*/true);
    NiceMock<test::MockDroneControl> drone_control;
    // Drone centered on the occupied voxel (55,55,55).
    EXPECT_CALL(drone_control, state()).WillRepeatedly(Return(stateAt(55, 55, 55)));
    EXPECT_CALL(drone_control, step()).Times(0);

    const auto dir = test::makeTempDir("mc_collide");
    MissionControlImpl mission_control(makeMission(10), makeDrone(5), *maps.hidden, *maps.output,
                                       drone_control, dir / "run" / "output_map.npy");

    const types::MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 0u);
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors.front().code, "INITIAL_DRONE_COLLIDES_WITH_OCCUPIED_VOXEL");
}

TEST(MissionControl, InFlightCollisionErrorsAfterStep) {
    Maps maps = makeMaps(/*occupied_center=*/true);
    NiceMock<test::MockDroneControl> drone_control;
    // Preflight at a safe cell, then the executed step lands on the obstacle.
    EXPECT_CALL(drone_control, state())
        .WillOnce(Return(stateAt(5, 5, 5)))
        .WillOnce(Return(stateAt(55, 55, 55)));
    EXPECT_CALL(drone_control, step()).WillOnce(Return(stepContinue()));

    const auto dir = test::makeTempDir("mc_inflight");
    MissionControlImpl mission_control(makeMission(10), makeDrone(/*radius=*/1), *maps.hidden,
                                       *maps.output, drone_control, dir / "run" / "output_map.npy");

    const types::MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 1u);
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors.front().code, "DRONE_HITS_OBSTACLE");
}

TEST(MissionControl, DroneStepErrorIsReported) {
    Maps maps = makeMaps(false);
    NiceMock<test::MockDroneControl> drone_control;
    EXPECT_CALL(drone_control, state()).WillRepeatedly(Return(stateAt(50, 50, 50)));
    EXPECT_CALL(drone_control, step()).WillOnce(Return(stepError("movement driver exploded")));

    const auto dir = test::makeTempDir("mc_steperr");
    MissionControlImpl mission_control(makeMission(10), makeDrone(5), *maps.hidden, *maps.output,
                                       drone_control, dir / "run" / "output_map.npy");

    const types::MissionRunResult result = mission_control.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 1u);
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors.front().code, "DRONE_STEP_FAILED");
}

} // namespace
} // namespace drone_mapper
