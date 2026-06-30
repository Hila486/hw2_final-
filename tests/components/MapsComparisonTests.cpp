#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>

#include "support/TestHelpers.h"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace drone_mapper {
namespace {

using test::fullConfig;
using test::makeMapArray;
using test::setVoxelRaw;

// Builds a d=2,h=4,w=4 map (32 voxels): the whole z=0 layer is Occupied (1)
// and the z=1 layer is Empty (0). Sampling on this grid is 1:1 with voxels.
constexpr std::size_t kDepth = 2;
constexpr std::size_t kHeight = 4;
constexpr std::size_t kWidth = 4;
constexpr double kRes = 10.0;

[[nodiscard]] std::shared_ptr<NpyArray> makeHalfOccupiedArray() {
    auto array = makeMapArray(kDepth, kHeight, kWidth, 0);
    for (std::size_t y = 0; y < kHeight; ++y) {
        for (std::size_t x = 0; x < kWidth; ++x) {
            setVoxelRaw(*array, kHeight, kWidth, /*z=*/0, y, x, 1);
        }
    }
    return array;
}

[[nodiscard]] double scoreAgainst(const NpyArray& origin_template,
                                  std::shared_ptr<NpyArray> target_array) {
    auto origin_copy = makeMapArray(kDepth, kHeight, kWidth, 0);
    for (std::size_t i = 0; i < origin_template.NumValue(); ++i) {
        origin_copy->Data<int>()[i] = origin_template.Data<int>()[i];
    }

    const types::MapConfig config = fullConfig(kDepth, kHeight, kWidth, kRes);
    Map3DImpl origin(origin_copy, config);
    Map3DImpl target(std::move(target_array), config);

    const std::vector<IMap3D*> targets{&target};
    const std::vector<double> scores = MapsComparison::compare(origin, targets);
    EXPECT_EQ(scores.size(), 1u);
    return scores.front();
}

TEST(MapsComparison, IdenticalMapsScoreOneHundred) {
    auto origin = makeHalfOccupiedArray();
    auto target = makeHalfOccupiedArray();
    EXPECT_DOUBLE_EQ(scoreAgainst(*origin, std::move(target)), 100.0);
}

TEST(MapsComparison, VerySimilarMapsScoreCloseToButBelowOneHundred) {
    auto origin = makeHalfOccupiedArray();
    auto target = makeHalfOccupiedArray();
    // Flip exactly one of the 32 voxels -> 31/32 = 96.875.
    setVoxelRaw(*target, kHeight, kWidth, /*z=*/0, /*y=*/0, /*x=*/0, 0);

    const double score = scoreAgainst(*origin, std::move(target));
    EXPECT_GT(score, 90.0);
    EXPECT_LT(score, 100.0);
    EXPECT_NEAR(score, 96.875, 1.0e-6);
}

TEST(MapsComparison, OppositeMapsScoreCloseToZero) {
    auto origin = makeHalfOccupiedArray();
    // Invert every voxel -> nothing matches -> 0.
    auto target = makeMapArray(kDepth, kHeight, kWidth, 0);
    for (std::size_t i = 0; i < target->NumValue(); ++i) {
        target->Data<int>()[i] = origin->Data<int>()[i] == 1 ? 0 : 1;
    }

    const double score = scoreAgainst(*origin, std::move(target));
    EXPECT_LT(score, 5.0);
    EXPECT_DOUBLE_EQ(score, 0.0);
}

TEST(MapsComparison, PartialMatchScoresReasonableMiddle) {
    auto origin = makeHalfOccupiedArray();
    // All-empty target matches only the empty z=1 layer -> 16/32 = 50.
    auto target = makeMapArray(kDepth, kHeight, kWidth, 0);

    const double score = scoreAgainst(*origin, std::move(target));
    EXPECT_NEAR(score, 50.0, 1.0e-6);
}

TEST(MapsComparison, NullTargetScoresZeroWithoutCrashing) {
    auto origin_array = makeHalfOccupiedArray();
    const types::MapConfig config = fullConfig(kDepth, kHeight, kWidth, kRes);
    Map3DImpl origin(std::move(origin_array), config);

    const std::vector<IMap3D*> targets{nullptr};
    const std::vector<double> scores = MapsComparison::compare(origin, targets);

    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores.front(), 0.0);
}

TEST(MapsComparison, ReturnsOneScorePerTarget) {
    auto origin_array = makeHalfOccupiedArray();
    auto identical_array = makeHalfOccupiedArray();
    auto empty_array = makeMapArray(kDepth, kHeight, kWidth, 0);

    const types::MapConfig config = fullConfig(kDepth, kHeight, kWidth, kRes);
    Map3DImpl origin(std::move(origin_array), config);
    Map3DImpl identical(std::move(identical_array), config);
    Map3DImpl empty(std::move(empty_array), config);

    const std::vector<IMap3D*> targets{&identical, &empty};
    const std::vector<double> scores = MapsComparison::compare(origin, targets);

    ASSERT_EQ(scores.size(), 2u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
    EXPECT_NEAR(scores[1], 50.0, 1.0e-6);
}

} // namespace
} // namespace drone_mapper
