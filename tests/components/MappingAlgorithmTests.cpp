#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>

#include "support/TestHelpers.h"

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <utility>
#include <vector>

namespace drone_mapper {
namespace {

using test::fullConfig;
using test::heading;
using test::makeDrone;
using test::makeLidar;
using test::makeMapArray;
using test::makeMission;
using test::P;

// The fresh-output-map planning grid: Unmapped voxels are legal to traverse.
[[nodiscard]] std::unique_ptr<Map3DImpl>
makeFreshOutputMap(std::size_t depth, std::size_t height, std::size_t width, double res) {
    return std::make_unique<Map3DImpl>(
        makeMapArray(depth, height, width, static_cast<int>(types::VoxelOccupancy::Unmapped)),
        fullConfig(depth, height, width, res));
}

// Drives nextStep from a fixed drone state, feeding a (dummy) scan result after
// the first call, and stopping once the algorithm issues a non-scan action.
[[nodiscard]] std::vector<types::MappingStepCommand>
driveUntilAction(IMappingAlgorithm& algorithm, const types::DroneState& state, int max_calls = 20) {
    std::vector<types::MappingStepCommand> commands;
    const types::LidarScanResult dummy_scan;
    const types::LidarScanResult* feed = nullptr;

    for (int call = 0; call < max_calls; ++call) {
        const types::MappingStepCommand command = algorithm.nextStep(state, feed);
        commands.push_back(command);
        const bool is_scan = command.scan_orientation.has_value() && !command.movement.has_value();
        if (!is_scan) {
            break;
        }
        feed = &dummy_scan;
    }
    return commands;
}

// A drone with radius 0 keeps the legality check simple (no sphere sampling),
// and a 10cm planning step (one voxel) keeps grid math predictable.
[[nodiscard]] types::DroneConfigData planningDrone() {
    return makeDrone(/*radius=*/0.0, /*max_rotate=*/90.0, /*max_advance=*/10.0,
                     /*max_elevate=*/10.0);
}

TEST(MappingAlgorithm, FirstStepIsAScanWithoutMovement) {
    auto output = makeFreshOutputMap(1, 5, 5, 10.0);
    MappingAlgorithmImpl algorithm(makeMission(), makeLidar(), planningDrone(), *output);

    const types::MappingStepCommand command =
        algorithm.nextStep(types::DroneState{P(25, 25, 5), heading(0), 0}, nullptr);

    EXPECT_EQ(command.status, types::AlgorithmStatus::Working);
    ASSERT_TRUE(command.scan_orientation.has_value());
    EXPECT_FALSE(command.movement.has_value());
    EXPECT_DOUBLE_EQ(command.scan_orientation->horizontal.force_numerical_value_in(deg), 0.0);
    EXPECT_DOUBLE_EQ(command.scan_orientation->altitude.force_numerical_value_in(deg), 0.0);
}

TEST(MappingAlgorithm, ScansSixDistinctDirectionsBeforeMoving) {
    auto output = makeFreshOutputMap(1, 5, 5, 10.0);
    MappingAlgorithmImpl algorithm(makeMission(), makeLidar(), planningDrone(), *output);

    const auto commands = driveUntilAction(algorithm, types::DroneState{P(25, 25, 5), heading(0), 0});

    ASSERT_GE(commands.size(), 7u);
    // First six commands must be the six scan directions, in order.
    const std::array<std::pair<double, double>, 6> expected{{
        {0.0, 0.0}, {-90.0, 0.0}, {180.0, 0.0}, {90.0, 0.0}, {0.0, 90.0}, {0.0, -90.0},
    }};
    for (std::size_t i = 0; i < expected.size(); ++i) {
        ASSERT_TRUE(commands[i].scan_orientation.has_value()) << "command " << i;
        EXPECT_FALSE(commands[i].movement.has_value()) << "command " << i;
        EXPECT_EQ(commands[i].status, types::AlgorithmStatus::Working);
        EXPECT_DOUBLE_EQ(commands[i].scan_orientation->horizontal.force_numerical_value_in(deg),
                         expected[i].first)
            << "command " << i;
        EXPECT_DOUBLE_EQ(commands[i].scan_orientation->altitude.force_numerical_value_in(deg),
                         expected[i].second)
            << "command " << i;
    }

    // After scanning, it should choose to move into a free neighbouring cell.
    const types::MappingStepCommand& action = commands.back();
    ASSERT_TRUE(action.movement.has_value());
    EXPECT_EQ(action.status, types::AlgorithmStatus::Working);
}

TEST(MappingAlgorithm, FinishesWhenNoLegalNeighbourExists) {
    // A single-voxel world: every neighbour is out of bounds, so after the scan
    // sweep the algorithm must report Finished.
    auto output = makeFreshOutputMap(1, 1, 1, 10.0);
    MappingAlgorithmImpl algorithm(makeMission(), makeLidar(), planningDrone(), *output);

    const auto commands = driveUntilAction(algorithm, types::DroneState{P(5, 5, 5), heading(0), 0});

    ASSERT_FALSE(commands.empty());
    const types::MappingStepCommand& last = commands.back();
    EXPECT_FALSE(last.movement.has_value());
    EXPECT_EQ(last.status, types::AlgorithmStatus::Finished);
}

TEST(MappingAlgorithm, DoesNotPlanThroughOccupiedNeighbours) {
    // Surround the drone cell with occupied voxels in the output map; the only
    // open neighbour is to the east, so the eventual move must head +x.
    auto output_array = makeMapArray(1, 3, 3, static_cast<int>(types::VoxelOccupancy::Occupied));
    // Free the drone cell (center) and only the east neighbour.
    test::setVoxelRaw(*output_array, 3, 3, 0, 1, 1, static_cast<int>(types::VoxelOccupancy::Empty));
    test::setVoxelRaw(*output_array, 3, 3, 0, 1, 2, static_cast<int>(types::VoxelOccupancy::Empty));
    Map3DImpl output(std::move(output_array), fullConfig(1, 3, 3, 10.0));

    MappingAlgorithmImpl algorithm(makeMission(), makeLidar(), planningDrone(), output);

    const auto commands = driveUntilAction(algorithm, types::DroneState{P(15, 15, 5), heading(0), 0});

    const types::MappingStepCommand& action = commands.back();
    ASSERT_TRUE(action.movement.has_value());
    // East is reachable with heading 0 already, so the move is a straight advance.
    EXPECT_EQ(action.movement->type, types::MovementCommandType::Advance);
}

} // namespace
} // namespace drone_mapper
