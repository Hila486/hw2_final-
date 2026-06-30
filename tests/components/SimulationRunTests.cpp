#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include "support/TestHelpers.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <utility>

namespace drone_mapper {
namespace {

using ::testing::NiceMock;
using ::testing::Return;

using test::fullConfig;
using test::heading;
using test::L;
using test::makeDrone;
using test::makeLidar;
using test::makeMapArray;
using test::makeMission;
using test::P;
using test::setVoxelRaw;

constexpr std::size_t kN = 2; // 2x2x2 maps -> 8 sampled voxels
constexpr double kRes = 10.0;

// Builds and runs a SimulationRunImpl with mocked surrounding components, so the
// test isolates run()'s own logic (resolution policy + map scoring + result
// assembly). The GPS/movement/lidar/algorithm/drone-control are unused by run()
// but must be non-null for construction.
[[nodiscard]] types::SimulationResult runWith(std::shared_ptr<NpyArray> hidden_array,
                                              std::shared_ptr<NpyArray> output_array,
                                              const types::MissionConfigData& mission,
                                              const types::MissionRunResult& mission_result,
                                              const std::filesystem::path& output_file) {
    const types::MapConfig config = fullConfig(kN, kN, kN, kRes);

    auto hidden = std::make_unique<Map3DImpl>(std::move(hidden_array), config);
    auto output = std::make_unique<Map3DImpl>(std::move(output_array), config);
    auto gps = std::make_unique<NiceMock<test::MockGpsInterface>>();
    auto movement = std::make_unique<NiceMock<test::MockMovementInterface>>();
    auto lidar = std::make_unique<NiceMock<test::MockLidarInterface>>();
    auto algorithm = std::make_unique<NiceMock<test::MockMappingAlgorithm>>(
        mission, makeLidar(), makeDrone(), *output);
    auto drone_control = std::make_unique<NiceMock<test::MockDroneControl>>();

    auto mission_control = std::make_unique<NiceMock<test::MockMissionControl>>();
    ON_CALL(*mission_control, runMission()).WillByDefault(Return(mission_result));

    SimulationRunImpl run(std::move(hidden),
                          std::move(output),
                          std::move(gps),
                          std::move(movement),
                          std::move(lidar),
                          std::move(algorithm),
                          std::move(drone_control),
                          std::move(mission_control),
                          types::SimulationConfigData{},
                          mission,
                          output_file);
    return run.run();
}

[[nodiscard]] types::MissionRunResult completedMission() {
    return types::MissionRunResult{types::MissionRunStatus::Completed, 12, {}};
}

// --- SimulationRunImpl::run() -------------------------------------------------

TEST(SimulationRun, IdenticalMapsScoreOneHundredOnCompletedMission) {
    const auto dir = test::makeTempDir("run_identical");
    const types::SimulationResult result =
        runWith(makeMapArray(kN, kN, kN, 0), makeMapArray(kN, kN, kN, 0), makeMission(),
                completedMission(), dir / "output_map.npy");

    EXPECT_DOUBLE_EQ(result.mission_score, 100.0);
    ASSERT_FALSE(result.mission_results.empty());
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.mission_results.front().steps, 12u);
}

TEST(SimulationRun, DifferingMapsScoreBelowOneHundred) {
    auto output_array = makeMapArray(kN, kN, kN, 0);
    setVoxelRaw(*output_array, kN, kN, /*z=*/0, /*y=*/0, /*x=*/0, 1); // one wrong voxel of 8

    const auto dir = test::makeTempDir("run_differ");
    const types::SimulationResult result =
        runWith(makeMapArray(kN, kN, kN, 0), std::move(output_array), makeMission(),
                completedMission(), dir / "output_map.npy");

    EXPECT_NEAR(result.mission_score, 87.5, 1.0e-6); // 7/8 voxels match
}

TEST(SimulationRun, MissionErrorScoresMinusOneAndSkipsComparison) {
    const auto dir = test::makeTempDir("run_error");
    const types::MissionRunResult error_mission{
        types::MissionRunStatus::Error, 3, {types::ErrorRef{"DRONE_HITS_OBSTACLE", "boom"}}};

    // Even though both maps are identical (would score 100), an errored mission
    // must score -1.
    const types::SimulationResult result =
        runWith(makeMapArray(kN, kN, kN, 0), makeMapArray(kN, kN, kN, 0), makeMission(),
                error_mission, dir / "output_map.npy");

    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
}

TEST(SimulationRun, ResolutionFactorOfOneIsAccepted) {
    const auto dir = test::makeTempDir("run_res_accepted");
    const types::SimulationResult result =
        runWith(makeMapArray(kN, kN, kN, 0), makeMapArray(kN, kN, kN, 0), makeMission(50, 10, 1.0),
                completedMission(), dir / "output_map.npy");
    EXPECT_EQ(result.resolution_request_status, types::ResolutionRequestStatus::Accepted);
}

TEST(SimulationRun, ResolutionFactorGreaterThanOneIsIgnored) {
    const auto dir = test::makeTempDir("run_res_ignored");
    const types::SimulationResult result =
        runWith(makeMapArray(kN, kN, kN, 0), makeMapArray(kN, kN, kN, 0), makeMission(50, 10, 2.0),
                completedMission(), dir / "output_map.npy");
    EXPECT_EQ(result.resolution_request_status, types::ResolutionRequestStatus::Ignored);
}

TEST(SimulationRun, ResolutionFactorLessThanOneIsIgnoredTooSmall) {
    const auto dir = test::makeTempDir("run_res_too_small");
    const types::SimulationResult result =
        runWith(makeMapArray(kN, kN, kN, 0), makeMapArray(kN, kN, kN, 0), makeMission(50, 10, 0.5),
                completedMission(), dir / "output_map.npy");
    EXPECT_EQ(result.resolution_request_status, types::ResolutionRequestStatus::IgnoredTooSmall);
}

TEST(SimulationRun, ConstructorRejectsNullDependency) {
    const types::MapConfig config = fullConfig(kN, kN, kN, kRes);
    const auto mission = makeMission();
    auto output_for_algo = std::make_unique<Map3DImpl>(makeMapArray(kN, kN, kN, 0), config);
    auto algorithm = std::make_unique<NiceMock<test::MockMappingAlgorithm>>(
        mission, makeLidar(), makeDrone(), *output_for_algo);

    EXPECT_THROW(
        SimulationRunImpl(std::make_unique<Map3DImpl>(makeMapArray(kN, kN, kN, 0), config),
                          /*output_map=*/nullptr,
                          std::make_unique<NiceMock<test::MockGpsInterface>>(),
                          std::make_unique<NiceMock<test::MockMovementInterface>>(),
                          std::make_unique<NiceMock<test::MockLidarInterface>>(),
                          std::move(algorithm),
                          std::make_unique<NiceMock<test::MockDroneControl>>(),
                          std::make_unique<NiceMock<test::MockMissionControl>>(),
                          types::SimulationConfigData{},
                          mission,
                          std::filesystem::path{}),
        std::invalid_argument);
}

// --- MockGPS -----------------------------------------------------------------

TEST(SimulationRun, MockGpsReturnsConstructedState) {
    MockGPS gps(P(10, 20, 30), heading(45, 10), L(5));

    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(gps.position().y.force_numerical_value_in(cm), 20.0);
    EXPECT_DOUBLE_EQ(gps.position().z.force_numerical_value_in(cm), 30.0);
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.force_numerical_value_in(deg), 45.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.force_numerical_value_in(deg), 10.0);
}

TEST(SimulationRun, MockGpsSettersUpdateState) {
    MockGPS gps(P(0, 0, 0), heading(0), L(5));

    gps.setPosition(P(1, 2, 3));
    gps.setHeading(heading(90, -5));

    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 1.0);
    EXPECT_DOUBLE_EQ(gps.position().y.force_numerical_value_in(cm), 2.0);
    EXPECT_DOUBLE_EQ(gps.position().z.force_numerical_value_in(cm), 3.0);
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.force_numerical_value_in(deg), 90.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.force_numerical_value_in(deg), -5.0);
}

// --- MockMovement ------------------------------------------------------------

TEST(SimulationRun, MockMovementAdvanceMovesAlongHeading) {
    MockGPS gps(P(0, 0, 0), heading(0), L(5)); // heading 0 == +x
    MockMovement movement(gps);

    const types::MovementResult result = movement.advance(L(10));

    EXPECT_TRUE(static_cast<bool>(result));
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 10.0, 1.0e-9);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 0.0, 1.0e-9);
    EXPECT_NEAR(gps.position().z.force_numerical_value_in(cm), 0.0, 1.0e-9);
}

TEST(SimulationRun, MockMovementAdvanceAtNinetyDegreesMovesAlongY) {
    MockGPS gps(P(0, 0, 0), heading(90), L(5));
    MockMovement movement(gps);

    movement.advance(L(10));

    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 0.0, 1.0e-6);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 10.0, 1.0e-6);
}

TEST(SimulationRun, MockMovementElevateChangesHeightOnly) {
    MockGPS gps(P(5, 5, 5), heading(0), L(5));
    MockMovement movement(gps);

    movement.elevate(L(15));
    EXPECT_NEAR(gps.position().z.force_numerical_value_in(cm), 20.0, 1.0e-9);

    movement.elevate(L(-8)); // elevate can be negative
    EXPECT_NEAR(gps.position().z.force_numerical_value_in(cm), 12.0, 1.0e-9);
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 5.0, 1.0e-9);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 5.0, 1.0e-9);
}

TEST(SimulationRun, MockMovementRotateChangesHeading) {
    MockGPS gps(P(0, 0, 0), heading(10), L(5));
    MockMovement movement(gps);

    movement.rotate(types::RotationDirection::Left, test::Hdeg(30));
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 40.0, 1.0e-9);

    movement.rotate(types::RotationDirection::Right, test::Hdeg(15));
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 25.0, 1.0e-9);
}

} // namespace
} // namespace drone_mapper
