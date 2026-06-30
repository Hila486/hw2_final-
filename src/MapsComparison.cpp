#include <drone_mapper/MapsComparison.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace drone_mapper {

namespace {

[[nodiscard]] double xCm(XLength value) { return value.force_numerical_value_in(cm); }
[[nodiscard]] double yCm(YLength value) { return value.force_numerical_value_in(cm); }
[[nodiscard]] double zCm(ZLength value) { return value.force_numerical_value_in(cm); }
[[nodiscard]] double pCm(PhysicalLength value) { return value.force_numerical_value_in(cm); }

// The comparison region is the world-space overlap of the two maps'
// boundaries. Boundaries already encode each map's axes offset, so they share
// a common world frame. When the output map carries mission/mapping bounds and
// the hidden map carries the full real extent, this overlap is exactly the
// mission region, which is what we want to score.
struct RegionCm {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    double min_z = 0.0;
    double max_z = 0.0;
    bool valid = false;
};

[[nodiscard]] RegionCm overlapRegion(const types::MapConfig& a, const types::MapConfig& b) {
    RegionCm region;
    region.min_x = std::max(xCm(a.boundaries.min_x), xCm(b.boundaries.min_x));
    region.max_x = std::min(xCm(a.boundaries.max_x), xCm(b.boundaries.max_x));
    region.min_y = std::max(yCm(a.boundaries.min_y), yCm(b.boundaries.min_y));
    region.max_y = std::min(yCm(a.boundaries.max_y), yCm(b.boundaries.max_y));
    region.min_z = std::max(zCm(a.boundaries.min_height), zCm(b.boundaries.min_height));
    region.max_z = std::min(zCm(a.boundaries.max_height), zCm(b.boundaries.max_height));
    region.valid = region.min_x <= region.max_x &&
                   region.min_y <= region.max_y &&
                   region.min_z <= region.max_z;
    return region;
}

// Number of voxel centers along one axis: centers sit at lo + (i + 0.5) * res.
[[nodiscard]] int centerCount(double lo, double hi, double res) {
    const double first_center = lo + 0.5 * res;
    if (first_center > hi) {
        return 0;
    }
    return static_cast<int>(std::floor((hi - first_center) / res + 1.0e-9)) + 1;
}

[[nodiscard]] double scoreOne(const IMap3D& origin, const IMap3D& target) {
    const types::MapConfig origin_config = origin.getMapConfig();
    const types::MapConfig target_config = target.getMapConfig();

    const RegionCm region = overlapRegion(origin_config, target_config);
    if (!region.valid) {
        return 0.0;
    }

    // Sample on the ground-truth (origin) grid. If both resolutions match this
    // is a 1:1 voxel comparison; if the target is coarser/finer each origin
    // voxel is compared against the target voxel that covers its center.
    double resolution_cm = pCm(origin_config.resolution);
    if (resolution_cm <= 0.0) {
        resolution_cm = pCm(target_config.resolution);
    }
    if (resolution_cm <= 0.0) {
        return 0.0;
    }

    const int nx = centerCount(region.min_x, region.max_x, resolution_cm);
    const int ny = centerCount(region.min_y, region.max_y, resolution_cm);
    const int nz = centerCount(region.min_z, region.max_z, resolution_cm);

    long long total = 0;
    long long correct = 0;

    for (int iz = 0; iz < nz; ++iz) {
        const double cz = region.min_z + (static_cast<double>(iz) + 0.5) * resolution_cm;
        for (int iy = 0; iy < ny; ++iy) {
            const double cy = region.min_y + (static_cast<double>(iy) + 0.5) * resolution_cm;
            for (int ix = 0; ix < nx; ++ix) {
                const double cx = region.min_x + (static_cast<double>(ix) + 0.5) * resolution_cm;

                const Position3D position{
                    cx * x_extent[cm],
                    cy * y_extent[cm],
                    cz * z_extent[cm],
                };

                const types::VoxelOccupancy expected = origin.atVoxel(position);
                const types::VoxelOccupancy actual = target.atVoxel(position);

                ++total;
                if (expected == actual) {
                    ++correct;
                }
            }
        }
    }

    if (total == 0) {
        return 0.0;
    }

    return 100.0 * static_cast<double>(correct) / static_cast<double>(total);
}

} // namespace

std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                            const std::vector<IMap3D*> targets) {
    std::vector<double> scores;
    scores.reserve(targets.size());

    for (const IMap3D* target : targets) {
        if (target == nullptr) {
            scores.push_back(0.0);
            continue;
        }
        scores.push_back(scoreOne(origin, *target));
    }

    return scores;
}

} // namespace drone_mapper
