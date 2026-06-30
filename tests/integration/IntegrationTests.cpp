#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/NpyMapIO.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/SimulationRunImpl.h>

#include "support/TestHelpers.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <utility>
#include <vector>

namespace drone_mapper {
namespace {

using ::testing::_;
using ::testing::NiceMock;
using ::testing::Return;

using test::fullConfig;
using test::heading;
using test::Hdeg;
using test::L;
using test::makeDrone;
using test::makeLidar;
using test::makeMapArray;
using test::makeMission;
using test::P;
using test::setVoxelRaw;

// Integration test 1: the entire pipeline (SimulationManager -> factory ->
// SimulationRun -> MissionControl -> DroneControl -> real MappingAlgorithm +
// MockLidar/GPS/Movement + Map3D), driven from a real .npy map on disk.
TEST(Integration, FullPipelineWithRealAlgorithmProducesScoredOutputMap) {
    const auto work_dir = test::makeTempDir("integration_real");
    const std::filesystem::path map_path = work_dir / "hidden_map.npy";

    // A small, fully-empty, fully-mappable world (depth 1, 3x3 in x/y).
    {
        Map3DImpl writer(makeMapArray(1, 3, 3, 0), fullConfig(1, 3, 3, 10.0));
        writer.save(map_path);
    }

    types::SimulationConfigData simulation;
    simulation.map_filename = map_path;
    simulation.map_resolution = L(10.0);
    simulation.map_offset = Position3D{};
    simulation.initial_drone_position = P(15, 15, 5);
    simulation.initial_angle = Hdeg(0);

    types::SimulationCompositionData composition;
    composition.composition_file = "integration.yaml";
    composition.simulation_mission_groups.emplace_back(
        simulation, std::vector<types::MissionConfigData>{makeMission(/*max_steps=*/500)});
    composition.drones = {makeDrone(/*radius=*/1, 90, 10, 10)};
    composition.lidars = {makeLidar(20.0, 120.0, 2.5, /*fov_circles=*/3)};

    SimulationManager manager(std::make_unique<SimulationRunFactoryImpl>());
    const types::SimulationManagerReport report = manager.run(composition, work_dir);

    ASSERT_EQ(report.runs.size(), 1u);
    const types::SimulationResult& run = report.runs.front();

    ASSERT_EQ(run.mission_results.size(), 1u);
    EXPECT_NE(run.mission_results.front().status, types::MissionRunStatus::Error);

    // A real run on a real map must produce a sensible accuracy score and a
    // valid output .npy of the same shape as the input.
    EXPECT_GT(run.mission_score, 0.0);
    EXPECT_LE(run.mission_score, 100.0);

    ASSERT_TRUE(std::filesystem::exists(run.output_map_file));
    auto output_array = loadNormalizedNpyMap(run.output_map_file);
    const NpyMapShape shape = npyMapShape(*output_array, run.output_map_file);
    EXPECT_EQ(shape.depth, 1u);
    EXPECT_EQ(shape.height, 3u);
    EXPECT_EQ(shape.width, 3u);
}

// Integration test 2: all real components wired together, but with a mock
// mapping algorithm scripting a deterministic scan-then-finish flow.
TEST(Integration, FullChainWithMockAlgorithmMapsScannedObstacle) {
    const double res = 10.0;
    const auto config = fullConfig(1, 1, 10, res);
    const auto mission = makeMission(/*max_steps=*/10);
    const auto drone = makeDrone(/*radius=*/1);

    // Hidden corridor with an obstacle at x index 5 (world voxel [50,60]).
    auto hidden_array = makeMapArray(1, 1, 10, 0);
    setVoxelRaw(*hidden_array, 1, 10, 0, 0, 5, 1);

    auto hidden = std::make_unique<Map3DImpl>(std::move(hidden_array), config);
    auto output = std::make_unique<Map3DImpl>(
        makeMapArray(1, 1, 10, static_cast<int>(types::VoxelOccupancy::Unmapped)), config);
    Map3DImpl* output_ptr = output.get();

    auto gps = std::make_unique<MockGPS>(P(5, 5, 5), heading(0), L(res));
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar = std::make_unique<MockLidar>(makeLidar(20.0, 120.0, 2.5, 1), *hidden, *gps);
    auto algorithm =
        std::make_unique<NiceMock<test::MockMappingAlgorithm>>(mission, makeLidar(), drone, *output);

    types::MappingStepCommand scan_command;
    scan_command.scan_orientation = heading(0);
    types::MappingStepCommand finish_command;
    finish_command.status = types::AlgorithmStatus::Finished;
    EXPECT_CALL(*algorithm, nextStep(_, _))
        .WillOnce(Return(scan_command))
        .WillRepeatedly(Return(finish_command));

    auto drone_control = std::make_unique<DroneControlImpl>(drone, mission, *lidar, *gps, *movement,
                                                            *output, *algorithm);

    const auto out_dir = test::makeTempDir("integration_mock");
    const std::filesystem::path output_file = out_dir / "output_map.npy";
    auto mission_control = std::make_unique<MissionControlImpl>(mission, drone, *hidden, *output,
                                                               *drone_control, output_file);

    SimulationRunImpl run(std::move(hidden), std::move(output), std::move(gps), std::move(movement),
                          std::move(lidar), std::move(algorithm), std::move(drone_control),
                          std::move(mission_control), types::SimulationConfigData{}, mission,
                          output_file);

    const types::SimulationResult result = run.run();

    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.mission_results.front().steps, 2u);

    // The scanned obstacle must appear in the output map, and the score reflects
    // the correctly-mapped portion (6/10 voxels: empties + the obstacle).
    EXPECT_EQ(output_ptr->atVoxel(P(55, 5, 5)), types::VoxelOccupancy::Occupied);
    EXPECT_NEAR(result.mission_score, 60.0, 5.0);
    EXPECT_TRUE(std::filesystem::exists(output_file));
}

} // namespace
} // namespace drone_mapper
