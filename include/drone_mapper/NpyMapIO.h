#pragma once

#include <TinyNPY.h>

#include <cstddef>
#include <filesystem>
#include <memory>

namespace drone_mapper {

// Shape of a 3D voxel map stored in an .npy array.
// The NPY layout is row-major (depth, height, width) == (z, y, x).
struct NpyMapShape {
    std::size_t depth = 0;
    std::size_t height = 0;
    std::size_t width = 0;
};

// Loads a 3D .npy map and normalizes it into an owned int32 array that
// Map3DImpl can read. Supports signed/unsigned/bool integer NPY dtypes.
// Throws std::runtime_error on any problem (missing file, wrong layout,
// non-3D shape, unsupported dtype).
[[nodiscard]] std::shared_ptr<NpyArray>
loadNormalizedNpyMap(const std::filesystem::path& map_filename);

// Returns the (depth, height, width) shape of a loaded array, validating that
// it is a non-empty row-major 3D array whose element count matches the shape.
[[nodiscard]] NpyMapShape npyMapShape(const NpyArray& map,
                                      const std::filesystem::path& source);

// Creates an owned int32 array of the given shape filled with a single value.
// Used for fresh output maps (filled with VoxelOccupancy::Unmapped).
[[nodiscard]] std::shared_ptr<NpyArray>
makeFilledIntNpyArray(const NpyMapShape& shape, int fill_value);

// Validates that an input (hidden) map only contains legal occupancy values:
// 0 (empty) and 1 (occupied). Any other value throws std::runtime_error.
// Output maps may contain Unmapped/PotentiallyOccupied and must NOT be checked
// with this function.
void validateInputMapValues(const NpyArray& map, const std::filesystem::path& source);

} // namespace drone_mapper
