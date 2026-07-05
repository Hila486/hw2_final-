#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <cstddef>
#include <optional>
#include <set>
#include <vector>

namespace drone_mapper {

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    //explicit MappingAlgorithmImpl(const types::DroneConfigData drone_config, const IMap3D& output_map);
    using IMappingAlgorithm::IMappingAlgorithm; // call the constructor of the base class
    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

private:
    struct GridCell {
        int x = 0;
        int y = 0;
        int z = 0;

        [[nodiscard]] bool operator<(const GridCell& other) const;
        [[nodiscard]] bool operator==(const GridCell& other) const;
    };

    struct PathNode {
        GridCell cell{};
        Position3D position{};
    };

    struct NavigationTarget {
        PathNode node{};
        bool backtracking = false;
        bool vertical = false;
        double desired_heading_degrees = 0.0;
        PhysicalLength distance{};
    };

    [[nodiscard]] GridCell cellFromPosition(const Position3D& position) const;
    [[nodiscard]] PhysicalLength planningStep() const;
    [[nodiscard]] bool isLegalDroneCenter(const Position3D& position) const;
    [[nodiscard]] std::optional<NavigationTarget> findExplorationTarget(
        const types::DroneState& state,
        const GridCell& current_cell) const;
    [[nodiscard]] types::MappingStepCommand commandForTarget(
        const types::DroneState& state,
        const GridCell& current_cell,
        const NavigationTarget& target);
    [[nodiscard]] types::MappingStepCommand scanCommand(std::size_t scan_index);
    [[nodiscard]] types::MappingStepCommand finishCommand() const;

    void initializeAtCurrentCell(const GridCell& current_cell, const Position3D& position);
    void handlePendingTranslationResult(const GridCell& current_cell);
    void resetScanSequenceForCell(const GridCell& current_cell);

    std::set<GridCell> visited_cells_;
    std::set<GridCell> attempted_cells_;
    std::vector<PathNode> dfs_path_;
    std::optional<GridCell> scan_cell_;
    std::size_t next_scan_index_ = 0;
    bool waiting_for_scan_result_ = false;
    std::optional<std::size_t> pending_scan_index_;
    std::optional<NavigationTarget> pending_target_;
    std::optional<NavigationTarget> pending_translation_;
    std::size_t failed_translation_count_ = 0;
};

} // namespace drone_mapper
