#include <drone_mapper/ScanResultToVoxels.h>
#include <drone_mapper/Map3DImpl.h>

#include "support/TestHelpers.h"

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <vector>

namespace drone_mapper {
namespace {

using test::fullConfig;
using test::heading;
using test::L;
using test::makeLidar;
using test::makeMapArray;
using test::P;

constexpr std::size_t kDepth = 1;
constexpr std::size_t kHeight = 1;
constexpr std::size_t kWidth = 10;
constexpr double kRes = 10.0;

TEST(ScanResultToVoxels, EmptyScanDoesNotChangeMap) {
    auto array = makeMapArray(kDepth, kHeight, kWidth, -1); // all Unmapped (-1)
    const types::MapConfig config = fullConfig(kDepth, kHeight, kWidth, kRes);
    Map3DImpl map(std::move(array), config);

    types::LidarScanResult scan; // empty scan

    ScanResultToVoxels::applyToMap(
        map,
        P(5, 5, 5),
        heading(0),
        scan,
        makeLidar()
    );

    // Verify all voxels remain Unmapped
    for (std::size_t x = 0; x < kWidth; ++x) {
        EXPECT_EQ(map.atVoxel(P(static_cast<double>(x) * kRes + 5.0, 5.0, 5.0)), types::VoxelOccupancy::Unmapped);
    }
}

TEST(ScanResultToVoxels, HitCreatesEmptyPathAndOccupiedVoxel) {
    auto array = makeMapArray(kDepth, kHeight, kWidth, -1); // all Unmapped
    const types::MapConfig config = fullConfig(kDepth, kHeight, kWidth, kRes);
    Map3DImpl map(std::move(array), config);

    types::LidarScanResult scan;
    // Hit at 45.0 cm facing heading 0 (along +x axis)
    types::LidarHit hit;
    hit.angle = heading(0);
    hit.distance = L(45.0);
    scan.push_back(hit);

    ScanResultToVoxels::applyToMap(
        map,
        P(5, 5, 5), // start at first voxel center
        heading(0),
        scan,
        makeLidar(20.0, 120.0, 2.5, 1)
    );

    // Hit is at x = 5.0 + 45.0 = 50.0 cm. Voxel 5 spans [50, 60], so it is Occupied.
    // Voxels before that (0, 1, 2, 3, 4) should be Empty.
    // Voxels after that (6, 7, 8, 9) should remain Unmapped.
    EXPECT_EQ(map.atVoxel(P(5.0, 5.0, 5.0)), types::VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(P(15.0, 5.0, 5.0)), types::VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(P(25.0, 5.0, 5.0)), types::VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(P(35.0, 5.0, 5.0)), types::VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(P(45.0, 5.0, 5.0)), types::VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(P(55.0, 5.0, 5.0)), types::VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(P(65.0, 5.0, 5.0)), types::VoxelOccupancy::Unmapped);
}

TEST(ScanResultToVoxels, MissCreatesEmptyPath) {
    auto array = makeMapArray(kDepth, kHeight, kWidth, -1);
    const types::MapConfig config = fullConfig(kDepth, kHeight, kWidth, kRes);
    Map3DImpl map(std::move(array), config);

    types::LidarScanResult scan;
    types::LidarHit hit;
    hit.angle = heading(0);
    hit.distance = L(std::numeric_limits<double>::max()); // Miss
    scan.push_back(hit);

    ScanResultToVoxels::applyToMap(
        map,
        P(5, 5, 5),
        heading(0),
        scan,
        makeLidar(20.0, 60.0, 2.5, 1) // z_max is 60.0
    );

    // Path up to 60.0cm should be Empty.
    // Starting at 5.0 cm, 5 + 60 = 65 cm (reaches voxel 6).
    // Voxel 0, 1, 2, 3, 4, 5, 6 should be Empty.
    // Voxels 7, 8, 9 remain Unmapped.
    EXPECT_EQ(map.atVoxel(P(5.0, 5.0, 5.0)), types::VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(P(55.0, 5.0, 5.0)), types::VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(P(65.0, 5.0, 5.0)), types::VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(P(75.0, 5.0, 5.0)), types::VoxelOccupancy::Unmapped);
}

TEST(ScanResultToVoxels, TooCloseHitCreatesPotentiallyOccupiedPath) {
    auto array = makeMapArray(kDepth, kHeight, kWidth, -1);
    const types::MapConfig config = fullConfig(kDepth, kHeight, kWidth, kRes);
    Map3DImpl map(std::move(array), config);

    types::LidarScanResult scan;
    types::LidarHit hit;
    hit.angle = heading(0);
    hit.distance = L(0.0); // Too close
    scan.push_back(hit);

    ScanResultToVoxels::applyToMap(
        map,
        P(5, 5, 5),
        heading(0),
        scan,
        makeLidar(40.0, 120.0, 2.5, 1) // z_min is 40.0
    );

    // Path up to z_min (40.0cm) should be PotentiallyOccupied.
    // Starting at 5.0 cm, 5 + 40 = 45 cm (reaches voxel 4).
    // Voxels 0, 1, 2, 3, 4 should be PotentiallyOccupied.
    // Voxels 5, 6, 7, 8, 9 remain Unmapped.
    EXPECT_EQ(map.atVoxel(P(5.0, 5.0, 5.0)), types::VoxelOccupancy::PotentiallyOccupied);
    EXPECT_EQ(map.atVoxel(P(45.0, 5.0, 5.0)), types::VoxelOccupancy::PotentiallyOccupied);
    EXPECT_EQ(map.atVoxel(P(55.0, 5.0, 5.0)), types::VoxelOccupancy::Unmapped);
}

TEST(ScanResultToVoxels, VoxelPriorityRules) {
    auto array = makeMapArray(kDepth, kHeight, kWidth, -1);
    const types::MapConfig config = fullConfig(kDepth, kHeight, kWidth, kRes);
    Map3DImpl map(std::move(array), config);

    // Initialize voxel at index 2 to Occupied
    map.set(P(25.0, 5.0, 5.0), types::VoxelOccupancy::Occupied);

    // Initialize voxel at index 3 to PotentiallyOccupied
    map.set(P(35.0, 5.0, 5.0), types::VoxelOccupancy::PotentiallyOccupied);

    // Apply a scan that would write Empty to index 2 and index 3
    types::LidarScanResult scan;
    types::LidarHit hit;
    hit.angle = heading(0);
    hit.distance = L(55.0); // covers up to index 5
    scan.push_back(hit);

    ScanResultToVoxels::applyToMap(
        map,
        P(5.0, 5.0, 5.0),
        heading(0),
        scan,
        makeLidar(20.0, 120.0, 2.5, 1)
    );

    // Voxel 2 (Occupied) should NOT be overridden by Empty (Occupied has higher priority)
    EXPECT_EQ(map.atVoxel(P(25.0, 5.0, 5.0)), types::VoxelOccupancy::Occupied);

    // Voxel 3 (PotentiallyOccupied) SHOULD be overridden by Empty (Empty has higher priority)
    EXPECT_EQ(map.atVoxel(P(35.0, 5.0, 5.0)), types::VoxelOccupancy::Empty);
}

} // namespace
} // namespace drone_mapper
