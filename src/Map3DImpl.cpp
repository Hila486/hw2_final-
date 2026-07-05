#include <drone_mapper/Map3DImpl.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

namespace drone_mapper {

namespace { // we have no acess to private member of the class here in the namespace, we do it to not change the header file.

struct MapShape { // struct to store the shape of the map
    std::size_t depth = 0;
    std::size_t height = 0;
    std::size_t width = 0;
};

struct VoxelIndex { // struct to hold the index of the voxel array
    std::size_t x = 0;
    std::size_t y = 0;
    std::size_t z = 0;
};

[[nodiscard]] bool getMapShape(const NpyArray& map, MapShape& shape) { // gets the shape of the map from the npy array
    if (map.IsEmpty() || map.ColMajor()) {
        return false;
    }

    const NpyArray::shape_t& npy_shape = map.Shape();
    if (npy_shape.size() != 3) {
        return false;
    }

    shape.depth = npy_shape[0];
    shape.height = npy_shape[1];
    shape.width = npy_shape[2];

    if (shape.depth == 0 || shape.height == 0 || shape.width == 0) {
        return false;
    }

    return map.NumValue() == shape.depth * shape.height * shape.width;
}

[[nodiscard]] std::size_t flatIndex(const VoxelIndex& index,
                                                     const MapShape& shape) { // converts voxel index to flat index of the voxel array
    return index.z * (shape.height * shape.width) + index.y * shape.width + index.x;
}

[[nodiscard]] bool positionToFlatIndex(const NpyArray& map,
                                                        const types::MapConfig& config,
                                                        const Position3D& pos,
                                                        std::size_t& output_index) { // converts position to flat index of the voxel array
    MapShape shape; // shape of the map
    if (!getMapShape(map, shape)) {
        return false;
    }

    const types::MappingBounds& bounds = config.boundaries; // map boundaries
    if (pos.x < bounds.min_x || pos.x > bounds.max_x ||
        pos.y < bounds.min_y || pos.y > bounds.max_y ||
        pos.z < bounds.min_height || pos.z > bounds.max_height) {
        return false;
    }

    const double resolution_cm = config.resolution.force_numerical_value_in(cm);
    if (resolution_cm <= 0.0) {
        return false;
    }

    const double map_x_cm = pos.x.force_numerical_value_in(cm) +
                            config.offset.x.force_numerical_value_in(cm);
    const double map_y_cm = pos.y.force_numerical_value_in(cm) +
                            config.offset.y.force_numerical_value_in(cm);
    const double map_z_cm = pos.z.force_numerical_value_in(cm) +
                            config.offset.z.force_numerical_value_in(cm);

    const double x_index_value = std::floor(map_x_cm / resolution_cm);
    const double y_index_value = std::floor(map_y_cm / resolution_cm);
    const double z_index_value = std::floor(map_z_cm / resolution_cm);

    if (x_index_value < 0.0 || y_index_value < 0.0 || z_index_value < 0.0) {
        return false;
    }

    if (x_index_value >= static_cast<double>(shape.width) ||
        y_index_value >= static_cast<double>(shape.height) ||
        z_index_value >= static_cast<double>(shape.depth)) {
        return false;
    }

    const VoxelIndex voxel_index{
        static_cast<std::size_t>(x_index_value),
        static_cast<std::size_t>(y_index_value),
        static_cast<std::size_t>(z_index_value),
    };

    output_index = flatIndex(voxel_index, shape);
    return true;
}

[[nodiscard]] types::VoxelOccupancy rawToVoxelOccupancy(int raw_value) {
    switch (raw_value) {
        case -3:
            return types::VoxelOccupancy::PotentiallyOccupied;
        case -2:
            return types::VoxelOccupancy::OutOfBounds;
        case -1:
            return types::VoxelOccupancy::Unmapped;
        case 0:
            return types::VoxelOccupancy::Empty;
        case 1:
            return types::VoxelOccupancy::Occupied;
        default:
            throw std::runtime_error("Invalid raw voxel occupancy value."); // if the raw value is not a valid voxel occupancy value, throw an error
    }
}

[[nodiscard]] int readRawValue(const NpyArray& map, std::size_t index) { // reads the raw value from the npy array
    const int raw_type = static_cast<int>(map.Type());
    const char type = static_cast<char>(raw_type < 0 ? -raw_type : raw_type);
    if (type != 'i' || map.SizeValueBytes() != sizeof(int)) {
        throw std::runtime_error("Unsupported NPY map data type.");
    }

    const int* data = map.Data<int>();
    if (data == nullptr) {
        throw std::runtime_error("Failed to access NPY map data.");
    }

    return data[index];
}

void writeRawValue(NpyArray& map, std::size_t index, int raw_value) { // writes the raw value to the npy array
    const int raw_type = static_cast<int>(map.Type());
    const char type = static_cast<char>(raw_type < 0 ? -raw_type : raw_type);
    if (type != 'i' || map.SizeValueBytes() != sizeof(int)) {
        throw std::runtime_error("Unsupported NPY map data type.");
    }

    int* data = map.Data<int>();
    if (data == nullptr) {
        throw std::runtime_error("Failed to access NPY map data.");
    }

    data[index] = raw_value;
}

} // namespace block end of the anonymous namespace

// if no map config is provided, the second constructor is called with default map config,
// otherwise the second constructor is called with the provided map config.
Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr) // first constructor.
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {} //  call the second constructor with default map config.

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const { // returns the voxel occupancy at the given position
    std::size_t index = 0;
    if (!positionToFlatIndex(*map_, config_, pos, index)) { // converts position to flat index of the voxel array
        return types::VoxelOccupancy::OutOfBounds;
    }

    return rawToVoxelOccupancy(readRawValue(*map_, index)); // converts raw value to voxel occupancy
}

types::MapConfig Map3DImpl::getMapConfig() const { // returns the map config- map metadata
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const { // checks if position is within the map bounds
    const types::MappingBounds& bounds = config_.boundaries;
    return pos.x >= bounds.min_x && pos.x <= bounds.max_x &&
           pos.y >= bounds.min_y && pos.y <= bounds.max_y &&
           pos.z >= bounds.min_height && pos.z <= bounds.max_height;
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) { // sets the voxel occupancy at the given position
    std::size_t index = 0;
    if (!positionToFlatIndex(*map_, config_, pos, index)) { // converts position to flat index of the voxel array
        return;
    }

    writeRawValue(*map_, index, static_cast<int>(value)); // writes the raw value to the npy array
}

void Map3DImpl::save(const std::filesystem::path& path) const { // saves the map to a file
    const std::filesystem::path parent_path = path.parent_path(); // get the parent path of the file
    if (!parent_path.empty()) { // if the parent path is not empty, create the directory
        std::filesystem::create_directories(parent_path); // create the directory
    }

    const char* error = map_->SaveNPY(path.string()); // save the map to the file
    if (error != nullptr) { // if the error is not null, throw an error
        throw std::runtime_error(std::string("Failed to save NPY file: ") + error);
    }
}

} // namespace drone_mapper
//saved
