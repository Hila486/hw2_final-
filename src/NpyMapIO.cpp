#include <drone_mapper/NpyMapIO.h>

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace drone_mapper {

namespace {

template <typename T>
void copyNpyValuesAsInt(const NpyArray& map, std::vector<int>& values) {
    const T* data = map.Data<T>();
    if (data == nullptr) {
        throw std::runtime_error("Failed to access NPY map data.");
    }

    values.resize(map.NumValue());
    for (std::size_t index = 0; index < map.NumValue(); ++index) {
        values[index] = static_cast<int>(data[index]);
    }
}

[[nodiscard]] std::vector<int> intValuesFromNpy(const NpyArray& map,
                                                const std::filesystem::path& source) {
    const int raw_type = static_cast<int>(map.Type());
    const char type = static_cast<char>(raw_type < 0 ? -raw_type : raw_type);
    const std::size_t word_size = map.SizeValueBytes();

    std::vector<int> values;
    if (type == 'i') {
        if (word_size == 1) {
            copyNpyValuesAsInt<std::int8_t>(map, values);
            return values;
        }
        if (word_size == 2) {
            copyNpyValuesAsInt<std::int16_t>(map, values);
            return values;
        }
        if (word_size == 4) {
            copyNpyValuesAsInt<std::int32_t>(map, values);
            return values;
        }
    }

    if (type == 'u') {
        if (word_size == 1) {
            copyNpyValuesAsInt<std::uint8_t>(map, values);
            return values;
        }
        if (word_size == 2) {
            copyNpyValuesAsInt<std::uint16_t>(map, values);
            return values;
        }
        if (word_size == 4) {
            copyNpyValuesAsInt<std::uint32_t>(map, values);
            return values;
        }
    }

    if (type == 'b' && word_size == 1) {
        copyNpyValuesAsInt<std::uint8_t>(map, values);
        return values;
    }

    throw std::runtime_error("Unsupported NPY map data type: " + source.string());
}

[[nodiscard]] std::shared_ptr<NpyArray> createOwnedIntArray(const NpyArray::shape_t& shape,
                                                            const std::vector<int>& values) {
    if (shape.empty() || values.size() != NpyArray::NumValue(shape)) {
        throw std::runtime_error("NPY int array shape does not match value count.");
    }

    auto array = std::make_shared<NpyArray>(shape, sizeof(int), 'i');
    array->Allocate();

    int* output_data = array->Data<int>();
    if (output_data == nullptr) {
        throw std::runtime_error("Failed to allocate owned NPY int array.");
    }

    std::copy(values.begin(), values.end(), output_data);
    return array;
}

} // namespace

NpyMapShape npyMapShape(const NpyArray& map, const std::filesystem::path& source) {
    if (map.IsEmpty() || map.ColMajor()) {
        throw std::runtime_error("Invalid NPY map layout: " + source.string());
    }

    const NpyArray::shape_t& shape = map.Shape();
    if (shape.size() != 3 || shape[0] == 0 || shape[1] == 0 || shape[2] == 0) {
        throw std::runtime_error("Expected a non-empty 3D NPY map: " + source.string());
    }

    const NpyMapShape map_shape{shape[0], shape[1], shape[2]};
    if (map.NumValue() != map_shape.depth * map_shape.height * map_shape.width) {
        throw std::runtime_error("NPY map shape does not match element count: " + source.string());
    }

    return map_shape;
}

std::shared_ptr<NpyArray> loadNormalizedNpyMap(const std::filesystem::path& map_filename) {
    auto map = std::make_shared<NpyArray>();
    const char* error = map->LoadNPY(map_filename.string());
    if (error != nullptr) {
        throw std::runtime_error("Failed to load NPY map " + map_filename.string() + ": " + error);
    }

    const NpyMapShape shape = npyMapShape(*map, map_filename);
    const std::vector<int> normalized_values = intValuesFromNpy(*map, map_filename);
    const NpyArray::shape_t normalized_shape{shape.depth, shape.height, shape.width};
    return createOwnedIntArray(normalized_shape, normalized_values);
}

std::shared_ptr<NpyArray> makeFilledIntNpyArray(const NpyMapShape& shape, int fill_value) {
    const std::vector<int> values(shape.depth * shape.height * shape.width, fill_value);
    const NpyArray::shape_t array_shape{shape.depth, shape.height, shape.width};
    return createOwnedIntArray(array_shape, values);
}

void validateInputMapValues(const NpyArray& map, const std::filesystem::path& source) {
    const int* data = map.Data<int>();
    if (data == nullptr) {
        throw std::runtime_error("Failed to access input map data: " + source.string());
    }
    for (std::size_t index = 0; index < map.NumValue(); ++index) {
        const int value = data[index];
        if (value != 0 && value != 1) {
            throw std::runtime_error(
                "Invalid input map value " + std::to_string(value) + " in " + source.string() +
                ": only 0 (empty) and 1 (occupied) are allowed.");
        }
    }
}

} // namespace drone_mapper
