#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/NpyMapIO.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace drone_mapper {

namespace {

std::size_t nextRunId() { // generate a unique run ids
    static std::size_t next_id = 0;
    const std::size_t current_id = next_id;
    ++next_id;
    return current_id;
}

std::string makeRunFolderName(const std::filesystem::path& map_filename) { // make folder name for the run
    std::ostringstream stream;
    stream << "run_" << std::setw(4) << std::setfill('0') << nextRunId();

    const std::string map_stem = map_filename.stem().string();
    if (!map_stem.empty()) {
        stream << "_" << map_stem;
    }

    return stream.str();
}

std::filesystem::path makeOutputMapFile(const types::SimulationConfigData& simulation, // make output map file for the run
                                        const std::filesystem::path& output_path) {
    const std::filesystem::path run_folder =
        output_path / "output_results" / makeRunFolderName(simulation.map_filename);
    std::filesystem::create_directories(run_folder);
    return run_folder / "output_map.npy";
}

types::MapConfig mapConfigForShape(const NpyMapShape& shape,
                                   const types::SimulationConfigData& simulation) { // build the map config for the shape
    const double resolution_cm = simulation.map_resolution.force_numerical_value_in(cm);
    if (resolution_cm <= 0.0) {
        throw std::runtime_error("Simulation map resolution must be positive.");
    }

    const double offset_x_cm = simulation.map_offset.x.force_numerical_value_in(cm);
    const double offset_y_cm = simulation.map_offset.y.force_numerical_value_in(cm);
    const double offset_z_cm = simulation.map_offset.z.force_numerical_value_in(cm);

    /*
     * Real physical map extent derived from the NPY shape, resolution and axes
     * offset. These are the true outer faces of the map; Map3DImpl already maps
     * a position exactly on the far face to one-past-the-last voxel and reports
     * it OutOfBounds, so no epsilon shrink is needed here. Keeping the exact
     * extent lets the drone touch a boundary (tangency) without being rejected.
     */
    return types::MapConfig{
        types::MappingBounds{
            (-offset_x_cm) * x_extent[cm],
            (static_cast<double>(shape.width) * resolution_cm - offset_x_cm) * x_extent[cm],
            (-offset_y_cm) * y_extent[cm],
            (static_cast<double>(shape.height) * resolution_cm - offset_y_cm) * y_extent[cm],
            (-offset_z_cm) * z_extent[cm],
            (static_cast<double>(shape.depth) * resolution_cm - offset_z_cm) * z_extent[cm],
        },
        simulation.map_offset,
        simulation.map_resolution,
    };
}

} // namespace

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    const std::filesystem::path output_map_file = makeOutputMapFile(simulation, output_path);
    auto hidden_map_array = loadNormalizedNpyMap(simulation.map_filename);
    validateInputMapValues(*hidden_map_array, simulation.map_filename);
    const NpyMapShape map_shape = npyMapShape(*hidden_map_array, simulation.map_filename);
    
    /*const types::MapConfig map_config = mapConfigForShape(map_shape, simulation);
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_map_array, map_config);

    auto output_map = std::make_unique<Map3DImpl>(
        makeFilledIntNpyArray(map_shape, static_cast<int>(types::VoxelOccupancy::Unmapped)),
        map_config);*/


    const types::MapConfig map_config = mapConfigForShape(map_shape, simulation);
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_map_array, map_config);

    // Hidden map uses the full physical map.
    // Output map uses the mission boundaries, because only the mission area is scored/mapped.
    types::MapConfig output_map_config = map_config;
    output_map_config.boundaries = mission.mission_bounds;

    auto output_map = std::make_unique<Map3DImpl>(
        makeFilledIntNpyArray(map_shape, static_cast<int>(types::VoxelOccupancy::Unmapped)),
        output_map_config);



    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algorithm =
        std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone,
        mission,
        *lidar_impl,
        *gps,
        *movement,
        *output_map,
        *mapping_algorithm);

    auto mission_control = std::make_unique<MissionControlImpl>(
        mission,
        drone,
        *hidden_map,
        *output_map,
        *drone_control,
        output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file);
}

} // namespace drone_mapper
