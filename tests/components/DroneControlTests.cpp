#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>

#include "support/TestHelpers.h"

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace drone_mapper {
namespace {

using ::testing::_;
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

[[nodiscard]] types::MappingStepCommand finishCommand() {
    types::MappingStepCommand command;
    command.status = types::AlgorithmStatus::Finished;
    return command;
}

[[nodiscard]] types::MappingStepCommand advanceCommand(double distance_cm) {
    types::MovementCommand movement;
    movement.type = types::MovementCommandType::Advance;
    movement.distance = L(distance_cm);
    types::MappingStepCommand command;
    command.movement = movement;
    return command;
}

[[nodiscard]] types::MappingStepCommand rotateCommand(types::RotationDirection direction,
                                                      double angle_deg) {
    types::MovementCommand movement;
    movement.type = types::MovementCommandType::Rotate;
    movement.rotation = direction;
    movement.angle = test::Hdeg(angle_deg);
    types::MappingStepCommand command;
    command.movement = movement;
    return command;
}

[[nodiscard]] types::MappingStepCommand scanCommand() {
    types::MappingStepCommand command;
    command.scan_orientation = heading(0);
    return command;
}

[[nodiscard]] std::unique_ptr<Map3DImpl>
makeOutputMap(std::size_t depth, std::size_t height, std::size_t width, int fill, double res) {
    return std::make_unique<Map3DImpl>(makeMapArray(depth, height, width, fill),
                                       fullConfig(depth, height, width, res));
}

TEST(DroneControl, StateReflectsGpsAndStartsAtStepZero) {
    MockGPS gps(P(5, 5, 5), heading(45), L(10));
    MockMovement movement(gps);
    NiceMock<test::MockLidarInterface> lidar;
    auto output = makeOutputMap(10, 10, 10, 0, 10.0);
    NiceMock<test::MockMappingAlgorithm> algorithm(makeMission(), makeLidar(), makeDrone(), *output);

    DroneControlImpl drone_control(makeDrone(5), makeMission(), lidar, gps, movement, *output,
                                   algorithm);

    const types::DroneState state = drone_control.state();
    EXPECT_DOUBLE_EQ(state.position.x.force_numerical_value_in(cm), 5.0);
    EXPECT_DOUBLE_EQ(state.heading.horizontal.force_numerical_value_in(deg), 45.0);
    EXPECT_EQ(state.step_index, 0u);
}

TEST(DroneControl, FinishedCommandReturnsCompleted) {
    MockGPS gps(P(55, 55, 55), heading(0), L(10));
    MockMovement movement(gps);
    NiceMock<test::MockLidarInterface> lidar;
    auto output = makeOutputMap(10, 10, 10, 0, 10.0);
    NiceMock<test::MockMappingAlgorithm> algorithm(makeMission(), makeLidar(), makeDrone(), *output);
    EXPECT_CALL(algorithm, nextStep(_, _)).WillOnce(Return(finishCommand()));

    DroneControlImpl drone_control(makeDrone(1), makeMission(), lidar, gps, movement, *output,
                                   algorithm);

    EXPECT_EQ(drone_control.step().status, types::DroneStepStatus::Completed);
}

TEST(DroneControl, AdvanceThroughKnownEmptySpaceMovesDrone) {
    MockGPS gps(P(55, 55, 55), heading(0), L(10)); // heading 0 -> +x
    MockMovement movement(gps);
    NiceMock<test::MockLidarInterface> lidar;
    auto output = makeOutputMap(10, 10, 10, /*fill=Empty*/ 0, 10.0);
    NiceMock<test::MockMappingAlgorithm> algorithm(makeMission(), makeLidar(), makeDrone(), *output);
    EXPECT_CALL(algorithm, nextStep(_, _)).WillOnce(Return(advanceCommand(10.0)));

    DroneControlImpl drone_control(makeDrone(/*radius=*/1, 90, /*max_advance=*/20), makeMission(),
                                   lidar, gps, movement, *output, algorithm);

    const types::DroneStepResult result = drone_control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 65.0, 1.0e-6);
    EXPECT_EQ(drone_control.state().step_index, 1u);
}

TEST(DroneControl, AdvanceExceedingMaxIsRejected) {
    MockGPS gps(P(55, 55, 55), heading(0), L(10));
    MockMovement movement(gps);
    NiceMock<test::MockLidarInterface> lidar;
    auto output = makeOutputMap(10, 10, 10, 0, 10.0);
    NiceMock<test::MockMappingAlgorithm> algorithm(makeMission(), makeLidar(), makeDrone(), *output);
    EXPECT_CALL(algorithm, nextStep(_, _)).WillOnce(Return(advanceCommand(/*distance=*/100.0)));

    DroneControlImpl drone_control(makeDrone(1, 90, /*max_advance=*/20), makeMission(), lidar, gps,
                                   movement, *output, algorithm);

    const types::DroneStepResult result = drone_control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
    EXPECT_NE(result.message.find("max_advance"), std::string::npos);
    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 55.0); // did not move
}

TEST(DroneControl, RotateWithinLimitsUpdatesHeading) {
    MockGPS gps(P(55, 55, 55), heading(0), L(10));
    MockMovement movement(gps);
    NiceMock<test::MockLidarInterface> lidar;
    auto output = makeOutputMap(10, 10, 10, 0, 10.0);
    NiceMock<test::MockMappingAlgorithm> algorithm(makeMission(), makeLidar(), makeDrone(), *output);
    EXPECT_CALL(algorithm, nextStep(_, _))
        .WillOnce(Return(rotateCommand(types::RotationDirection::Left, 30.0)));

    DroneControlImpl drone_control(makeDrone(1, /*max_rotate=*/90), makeMission(), lidar, gps,
                                   movement, *output, algorithm);

    const types::DroneStepResult result = drone_control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 30.0, 1.0e-6);
}

TEST(DroneControl, AdvanceThroughUnknownSpaceIsRejected) {
    MockGPS gps(P(55, 55, 55), heading(0), L(10));
    MockMovement movement(gps);
    NiceMock<test::MockLidarInterface> lidar;
    // Unmapped output map -> the swept body is not known empty.
    auto output = makeOutputMap(10, 10, 10, static_cast<int>(types::VoxelOccupancy::Unmapped), 10.0);
    NiceMock<test::MockMappingAlgorithm> algorithm(makeMission(), makeLidar(), makeDrone(), *output);
    EXPECT_CALL(algorithm, nextStep(_, _)).WillOnce(Return(advanceCommand(10.0)));

    DroneControlImpl drone_control(makeDrone(1, 90, 20), makeMission(), lidar, gps, movement,
                                   *output, algorithm);

    const types::DroneStepResult result = drone_control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 55.0); // movement blocked
}

TEST(DroneControl, ScanWritesObstacleAndClearPathIntoOutputMap) {
    const double res = 10.0;
    // Hidden corridor with an obstacle at x index 5 (world voxel [50,60]).
    auto hidden_array = makeMapArray(1, 1, 10, 0);
    setVoxelRaw(*hidden_array, 1, 10, 0, 0, 5, 1);
    Map3DImpl hidden(std::move(hidden_array), fullConfig(1, 1, 10, res));

    MockGPS gps(P(5, 5, 5), heading(0), L(res));
    MockMovement movement(gps);
    MockLidar lidar(makeLidar(20.0, 120.0, 2.5, 1), hidden, gps);
    auto output =
        makeOutputMap(1, 1, 10, static_cast<int>(types::VoxelOccupancy::Unmapped), res);
    NiceMock<test::MockMappingAlgorithm> algorithm(makeMission(), makeLidar(), makeDrone(), *output);
    EXPECT_CALL(algorithm, nextStep(_, _)).WillOnce(Return(scanCommand()));

    DroneControlImpl drone_control(makeDrone(1), makeMission(), lidar, gps, movement, *output,
                                   algorithm);

    EXPECT_EQ(drone_control.step().status, types::DroneStepStatus::Continue);
    EXPECT_EQ(output->atVoxel(P(55, 5, 5)), types::VoxelOccupancy::Occupied); // the obstacle
    EXPECT_EQ(output->atVoxel(P(15, 5, 5)), types::VoxelOccupancy::Empty);    // cleared path
}

TEST(DroneControl, ScanResultIsOfferedToAlgorithmOnNextStepOnly) {
    const double res = 10.0;
    auto hidden_array = makeMapArray(1, 1, 10, 0);
    setVoxelRaw(*hidden_array, 1, 10, 0, 0, 5, 1);
    Map3DImpl hidden(std::move(hidden_array), fullConfig(1, 1, 10, res));

    MockGPS gps(P(5, 5, 5), heading(0), L(res));
    MockMovement movement(gps);
    MockLidar lidar(makeLidar(20.0, 120.0, 2.5, 1), hidden, gps);
    auto output =
        makeOutputMap(1, 1, 10, static_cast<int>(types::VoxelOccupancy::Unmapped), res);
    NiceMock<test::MockMappingAlgorithm> algorithm(makeMission(), makeLidar(), makeDrone(), *output);

    bool first_call_scan_was_null = false;
    bool second_call_scan_was_present = false;
    EXPECT_CALL(algorithm, nextStep(_, _))
        .WillOnce([&](const types::DroneState&, const types::LidarScanResult* scan) {
            first_call_scan_was_null = (scan == nullptr);
            return scanCommand();
        })
        .WillOnce([&](const types::DroneState&, const types::LidarScanResult* scan) {
            second_call_scan_was_present = (scan != nullptr);
            return finishCommand();
        });

    DroneControlImpl drone_control(makeDrone(1), makeMission(), lidar, gps, movement, *output,
                                   algorithm);

    (void)drone_control.step(); // issues the scan
    (void)drone_control.step(); // should receive the scan result

    EXPECT_TRUE(first_call_scan_was_null);
    EXPECT_TRUE(second_call_scan_was_present);
}

} // namespace
} // namespace drone_mapper
