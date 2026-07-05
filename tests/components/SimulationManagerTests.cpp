#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/SimulationManager.h>

#include "support/TestHelpers.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace drone_mapper {
namespace {

// A run that returns a canned score, or throws to exercise error resilience.
class FakeRun : public ISimulationRun {
public:
    FakeRun(double score, bool throw_in_run) : score_(score), throw_in_run_(throw_in_run) {}

    types::SimulationResult run() override {
        if (throw_in_run_) {
            throw std::runtime_error("simulated run failure");
        }
        types::SimulationResult result;
        result.mission_score = score_;
        result.mission_results = {
            types::MissionRunResult{types::MissionRunStatus::Completed, 7, {}}};
        return result;
    }

private:
    double score_;
    bool throw_in_run_;
};

// Counts create() calls and can be told to fail a specific run (either when
// creating it, or when its run() executes).
class FakeFactory : public ISimulationRunFactory {
public:
    int create_count = 0;
    int throw_create_at = -1;
    int throw_run_at = -1;
    double score = 42.0;

    std::unique_ptr<ISimulationRun> create(const types::SimulationConfigData&,
                                           const types::MissionConfigData&,
                                           const types::DroneConfigData&,
                                           const types::LidarConfigData&,
                                           const std::filesystem::path&) override {
        const int index = create_count++;
        if (index == throw_create_at) {
            throw std::runtime_error("simulated create failure");
        }
        return std::make_unique<FakeRun>(score, index == throw_run_at);
    }
};

[[nodiscard]] types::SimulationCompositionData
makeComposition(const std::vector<std::size_t>& missions_per_group,
                std::size_t drones,
                std::size_t lidars) {
    types::SimulationCompositionData composition;
    composition.composition_file = "composition.yaml";
    for (std::size_t mission_count : missions_per_group) {
        std::vector<types::MissionConfigData> missions(mission_count, test::makeMission());
        composition.simulation_mission_groups.emplace_back(types::SimulationConfigData{},
                                                           missions);
    }
    composition.drones.assign(drones, test::makeDrone());
    composition.lidars.assign(lidars, test::makeLidar());
    return composition;
}

TEST(SimulationManager, ConstructorRejectsNullFactory) {
    EXPECT_THROW(SimulationManager(nullptr), std::invalid_argument);
}

TEST(SimulationManager, RunsCartesianProductOfMissionsDronesLidars) {
    auto factory = std::make_unique<FakeFactory>();
    FakeFactory* factory_ptr = factory.get();
    SimulationManager manager(std::move(factory));

    // 1 simulation group with 2 missions, 2 drones, 2 lidars -> 8 runs.
    const auto composition = makeComposition({2}, 2, 2);
    const auto output = test::makeTempDir("manager_cartesian");

    const types::SimulationManagerReport report = manager.run(composition, output);

    EXPECT_EQ(factory_ptr->create_count, 8);
    EXPECT_EQ(report.runs.size(), 8u);
    for (const auto& run : report.runs) {
        EXPECT_DOUBLE_EQ(run.mission_score, 42.0);
    }
}

TEST(SimulationManager, IteratesEverySimulationGroup) {
    auto factory = std::make_unique<FakeFactory>();
    FakeFactory* factory_ptr = factory.get();
    SimulationManager manager(std::move(factory));

    // Two groups: 1 mission and 2 missions, 1 drone, 1 lidar -> 1 + 2 = 3 runs.
    const auto composition = makeComposition({1, 2}, 1, 1);
    const auto output = test::makeTempDir("manager_groups");

    const types::SimulationManagerReport report = manager.run(composition, output);

    EXPECT_EQ(factory_ptr->create_count, 3);
    EXPECT_EQ(report.runs.size(), 3u);
}

TEST(SimulationManager, FailingRunIsScoredMinusOneAndBatchContinues) {
    auto factory = std::make_unique<FakeFactory>();
    factory->throw_run_at = 1; // the second run's run() throws
    SimulationManager manager(std::move(factory));

    const auto composition = makeComposition({3}, 1, 1); // 3 runs
    const auto output = test::makeTempDir("manager_run_fail");

    const types::SimulationManagerReport report = manager.run(composition, output);

    ASSERT_EQ(report.runs.size(), 3u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, 42.0);
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, -1.0);
    EXPECT_DOUBLE_EQ(report.runs[2].mission_score, 42.0);

    ASSERT_FALSE(report.runs[1].mission_results.empty());
    EXPECT_EQ(report.runs[1].mission_results.front().status, types::MissionRunStatus::Error);
}

TEST(SimulationManager, FailingCreateIsScoredMinusOneAndBatchContinues) {
    auto factory = std::make_unique<FakeFactory>();
    factory->throw_create_at = 0; // creation of the first run throws
    SimulationManager manager(std::move(factory));

    const auto composition = makeComposition({2}, 1, 1); // 2 runs
    const auto output = test::makeTempDir("manager_create_fail");

    const types::SimulationManagerReport report = manager.run(composition, output);

    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, 42.0);
}

TEST(SimulationManager, ReportCarriesMetricAndScoreRange) {
    auto factory = std::make_unique<FakeFactory>();
    SimulationManager manager(std::move(factory));

    const auto composition = makeComposition({1}, 1, 1);
    const auto output = test::makeTempDir("manager_metadata");

    const types::SimulationManagerReport report = manager.run(composition, output);

    EXPECT_FALSE(report.generated_at_utc.empty());
    EXPECT_EQ(report.metric, "output_map_accuracy");
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 0.0);
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 100.0);
    EXPECT_EQ(report.error_score, -1);
}

} // namespace
} // namespace drone_mapper
