#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>

#include "support/TestHelpers.h"

#include <gtest/gtest.h>

#include <limits>
#include <memory>

namespace drone_mapper {
namespace {

using test::fullConfig;
using test::heading;
using test::L;
using test::makeLidar;
using test::makeMapArray;
using test::P;
using test::setVoxelRaw;

// A 1x1x10 corridor along +x (east). Voxels are 10cm; world x spans [0,100].
constexpr std::size_t kDepth = 1;
constexpr std::size_t kHeight = 1;
constexpr std::size_t kWidth = 10;
constexpr double kRes = 10.0;

[[nodiscard]] double cm_of(PhysicalLength length) {
    return length.force_numerical_value_in(cm);
}

[[nodiscard]] std::unique_ptr<Map3DImpl> makeCorridor(bool occupied_at_index_five) {
    auto array = makeMapArray(kDepth, kHeight, kWidth, 0);
    if (occupied_at_index_five) {
        setVoxelRaw(*array, kHeight, kWidth, /*z=*/0, /*y=*/0, /*x=*/5, 1);
    }
    return std::make_unique<Map3DImpl>(std::move(array), fullConfig(kDepth, kHeight, kWidth, kRes));
}

TEST(MockLidar, ConfigReturnsConfiguredValues) {
    auto map = makeCorridor(false);
    MockGPS gps(P(5, 5, 5), heading(0), L(kRes));
    const types::LidarConfigData input = makeLidar(20.0, 120.0, 2.5, 4);
    MockLidar lidar(input, *map, gps);

    const types::LidarConfigData output = lidar.config();
    EXPECT_DOUBLE_EQ(cm_of(output.z_min), 20.0);
    EXPECT_DOUBLE_EQ(cm_of(output.z_max), 120.0);
    EXPECT_DOUBLE_EQ(cm_of(output.d), 2.5);
    EXPECT_EQ(output.fov_circles, 4u);
}

TEST(MockLidar, ZeroFovCirclesReturnsEmptyScan) {
    auto map = makeCorridor(true);
    MockGPS gps(P(5, 5, 5), heading(0), L(kRes));
    MockLidar lidar(makeLidar(20.0, 120.0, 2.5, /*fov_circles=*/0), *map, gps);

    EXPECT_TRUE(lidar.scan(heading(0)).empty());
}

TEST(MockLidar, SingleCircleReturnsOnlyCenterBeam) {
    auto map = makeCorridor(false);
    MockGPS gps(P(5, 5, 5), heading(0), L(kRes));
    MockLidar lidar(makeLidar(20.0, 120.0, 2.5, /*fov_circles=*/1), *map, gps);

    const types::LidarScanResult result = lidar.scan(heading(0));
    ASSERT_EQ(result.size(), 1u);
    // The center beam carries the (relative) scan orientation unchanged.
    EXPECT_DOUBLE_EQ(result.front().angle.horizontal.force_numerical_value_in(deg), 0.0);
    EXPECT_DOUBLE_EQ(result.front().angle.altitude.force_numerical_value_in(deg), 0.0);
}

TEST(MockLidar, BeamCountGrowsWithFovCircles) {
    auto map = makeCorridor(false);
    MockGPS gps(P(5, 5, 5), heading(0), L(kRes));

    // beams = 1 (center) + sum_{c=1..fov-1} 4^c.
    MockLidar two(makeLidar(20.0, 120.0, 2.5, 2), *map, gps);
    EXPECT_EQ(two.scan(heading(0)).size(), 1u + 4u);

    MockLidar three(makeLidar(20.0, 120.0, 2.5, 3), *map, gps);
    EXPECT_EQ(three.scan(heading(0)).size(), 1u + 4u + 16u);
}

TEST(MockLidar, MissReturnsMaxDistance) {
    auto map = makeCorridor(/*occupied_at_index_five=*/false);
    MockGPS gps(P(5, 5, 5), heading(0), L(kRes));
    MockLidar lidar(makeLidar(20.0, 120.0, 2.5, 1), *map, gps);

    const types::LidarScanResult result = lidar.scan(heading(0));
    ASSERT_EQ(result.size(), 1u);
    EXPECT_DOUBLE_EQ(cm_of(result.front().distance), std::numeric_limits<double>::max());
}

TEST(MockLidar, DetectsObstacleStraightAhead) {
    auto map = makeCorridor(/*occupied_at_index_five=*/true);
    MockGPS gps(P(5, 5, 5), heading(0), L(kRes));
    MockLidar lidar(makeLidar(20.0, 120.0, 2.5, 1), *map, gps);

    const types::LidarScanResult result = lidar.scan(heading(0));
    ASSERT_EQ(result.size(), 1u);
    // Occupied voxel index 5 -> near face at x=50, origin x=5 -> distance ~45cm.
    const double distance_cm = cm_of(result.front().distance);
    EXPECT_LT(distance_cm, std::numeric_limits<double>::max());
    EXPECT_NEAR(distance_cm, 45.0, 1.5);
}

TEST(MockLidar, ObstacleBehindIsNotSeenWhenFacingAway) {
    auto map = makeCorridor(/*occupied_at_index_five=*/true);
    // Face west (+180deg) so the obstacle at +x is behind the drone.
    MockGPS gps(P(5, 5, 5), heading(180), L(kRes));
    MockLidar lidar(makeLidar(20.0, 120.0, 2.5, 1), *map, gps);

    const types::LidarScanResult result = lidar.scan(heading(0));
    ASSERT_EQ(result.size(), 1u);
    EXPECT_DOUBLE_EQ(cm_of(result.front().distance), std::numeric_limits<double>::max());
}

TEST(MockLidar, ObstacleBeyondMaxRangeIsNotDetected) {
    auto map = makeCorridor(/*occupied_at_index_five=*/true);
    MockGPS gps(P(5, 5, 5), heading(0), L(kRes));

    // Obstacle is around 45cm away, but z_max is only 30cm.
    MockLidar lidar(makeLidar(20.0, 30.0, 2.5, 1), *map, gps);

    const types::LidarScanResult result = lidar.scan(heading(0));

    ASSERT_EQ(result.size(), 1u);
    EXPECT_DOUBLE_EQ(cm_of(result.front().distance), std::numeric_limits<double>::max());
}

TEST(MockLidar, TooCloseObstacleReportsZeroDistance) {
    auto array = makeMapArray(kDepth, kHeight, kWidth, 0);

    // Origin is x=5cm. Voxel x=1 starts at x=10cm, so the hit is only ~5cm away,
    // below z_min=20cm.
    setVoxelRaw(*array, kHeight, kWidth, /*z=*/0, /*y=*/0, /*x=*/1, 1);

    auto map = std::make_unique<Map3DImpl>(
        std::move(array),
        fullConfig(kDepth, kHeight, kWidth, kRes));

    MockGPS gps(P(5, 5, 5), heading(0), L(kRes));
    MockLidar lidar(makeLidar(20.0, 120.0, 2.5, 1), *map, gps);

    const types::LidarScanResult result = lidar.scan(heading(0));

    ASSERT_EQ(result.size(), 1u);
    EXPECT_DOUBLE_EQ(cm_of(result.front().distance), 0.0);
}

} // namespace
} // namespace drone_mapper
