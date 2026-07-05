#include <drone_mapper/ConfigParser.h>

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_mapper {

namespace {

std::filesystem::path normalizedAbsolute(const std::filesystem::path& path) {
    return std::filesystem::absolute(path).lexically_normal();
}

std::filesystem::path resolvePath(const std::filesystem::path& path,
                                  const std::filesystem::path& fallback_base) {
    if (path.is_absolute()) {
        return path.lexically_normal();
    }

    const std::filesystem::path cwd_candidate = normalizedAbsolute(path);
    if (std::filesystem::exists(cwd_candidate)) {
        return cwd_candidate;
    }

    const std::filesystem::path base_candidate =
        (fallback_base / path).lexically_normal();
    if (std::filesystem::exists(base_candidate)) {
        return base_candidate;
    }

    return cwd_candidate;
}

std::string contextFor(const std::filesystem::path& filepath, const std::string& section) {
    return section + " in " + filepath.string();
}

YAML::Node requireNode(const YAML::Node& node,
                       const std::string& key,
                       const std::string& context) {
    const YAML::Node child = node[key];
    if (!child) {
        throw std::runtime_error("Missing required field '" + key + "' in " + context + ".");
    }
    return child;
}

template <typename T>
T requireValue(const YAML::Node& node, const std::string& key, const std::string& context) {
    try {
        return requireNode(node, key, context).as<T>();
    } catch (const YAML::Exception& exception) {
        throw std::runtime_error("Failed to parse field '" + key + "' in " + context +
                                 ": " + exception.what());
    }
}

template <typename T>
T optionalValue(const YAML::Node& node, const std::string& key, T default_value) {
    const YAML::Node child = node[key];
    if (!child) {
        return default_value;
    }
    return child.as<T>();
}

YAML::Node loadRoot(const std::filesystem::path& filepath) {
    try {
        return YAML::LoadFile(filepath.string());
    } catch (const YAML::Exception& exception) {
        throw std::runtime_error("Failed to load YAML file " + filepath.string() +
                                 ": " + exception.what());
    }
}

types::MappingBounds parseBounds(const YAML::Node& boundaries,
                                 const std::filesystem::path& filepath) {
    const std::string context = contextFor(filepath, "mission_config.boundaries");
    const YAML::Node x_boundary = requireNode(boundaries, "x_boundary", context);
    const YAML::Node y_boundary = requireNode(boundaries, "y_boundary", context);
    const YAML::Node height_boundary = requireNode(boundaries, "height_boundary", context);

    return types::MappingBounds{
        requireValue<double>(x_boundary, "min_cm", context + ".x_boundary") * x_extent[cm],
        requireValue<double>(x_boundary, "max_cm", context + ".x_boundary") * x_extent[cm],
        requireValue<double>(y_boundary, "min_cm", context + ".y_boundary") * y_extent[cm],
        requireValue<double>(y_boundary, "max_cm", context + ".y_boundary") * y_extent[cm],
        requireValue<double>(height_boundary, "min_cm", context + ".height_boundary") *
            z_extent[cm],
        requireValue<double>(height_boundary, "max_cm", context + ".height_boundary") *
            z_extent[cm],
    };
}

Position3D parsePosition(const YAML::Node& node,
                         const std::string& context) {
    return Position3D{
        requireValue<double>(node, "x_cm", context) * x_extent[cm],
        requireValue<double>(node, "y_cm", context) * y_extent[cm],
        requireValue<double>(node, "height_cm", context) * z_extent[cm],
    };
}

Position3D parseMapOffset(const YAML::Node& node) {
    if (!node) {
        return Position3D{};
    }

    return Position3D{
        optionalValue<double>(node, "x_offset", 0.0) * x_extent[cm],
        optionalValue<double>(node, "y_offset", 0.0) * y_extent[cm],
        optionalValue<double>(node, "height_offset", 0.0) * z_extent[cm],
    };
}

std::runtime_error wrapParseError(const std::filesystem::path& filepath,
                                  const std::exception& exception) {
    return std::runtime_error("YAML parsing error in " + filepath.string() + ": " +
                              exception.what());
}

} // namespace

types::DroneConfigData ConfigParser::parseDroneConfig(const std::filesystem::path& filepath) {
    try {
        const YAML::Node root = loadRoot(filepath);
        const YAML::Node config =
            requireNode(root, "drone_config", contextFor(filepath, "root"));
        const std::string context = contextFor(filepath, "drone_config");

        const double diameter_cm = requireValue<double>(config, "dimensions_cm", context);
        return types::DroneConfigData{
            (diameter_cm / 2.0) * cm,
            requireValue<double>(config, "max_rotate_deg", context) * horizontal_angle[deg],
            requireValue<double>(config, "max_advance_cm", context) * cm,
            requireValue<double>(config, "max_elevate_cm", context) * cm,
        };
    } catch (const std::exception& exception) {
        throw wrapParseError(filepath, exception);
    }
}

types::MissionConfigData ConfigParser::parseMissionConfig(const std::filesystem::path& filepath) {
    try {
        const YAML::Node root = loadRoot(filepath);
        const YAML::Node config =
            requireNode(root, "mission_config", contextFor(filepath, "root"));
        const std::string context = contextFor(filepath, "mission_config");

        return types::MissionConfigData{
            requireValue<std::size_t>(config, "max_steps", context),
            requireValue<double>(config, "gps_resolution_cm", context) * cm,
            optionalValue<double>(config, "output_mapping_resolution_factor", 1.0),
            parseBounds(requireNode(config, "boundaries", context), filepath),
        };
    } catch (const std::exception& exception) {
        throw wrapParseError(filepath, exception);
    }
}

types::LidarConfigData ConfigParser::parseLidarConfig(const std::filesystem::path& filepath) {
    try {
        const YAML::Node root = loadRoot(filepath);
        const YAML::Node config =
            requireNode(root, "lidar_config", contextFor(filepath, "root"));
        const std::string context = contextFor(filepath, "lidar_config");

        return types::LidarConfigData{
            requireValue<double>(config, "z_min_cm", context) * cm,
            requireValue<double>(config, "z_max_cm", context) * cm,
            requireValue<double>(config, "d_cm", context) * cm,
            requireValue<std::size_t>(config, "fov_circles", context),
        };
    } catch (const std::exception& exception) {
        throw wrapParseError(filepath, exception);
    }
}

types::SimulationConfigData
ConfigParser::parseSimulationConfig(const std::filesystem::path& filepath) {
    try {
        const YAML::Node root = loadRoot(filepath);
        const YAML::Node config =
            requireNode(root, "simulation_config", contextFor(filepath, "root"));
        const std::string context = contextFor(filepath, "simulation_config");

        const std::filesystem::path map_filename = resolvePath(
            requireValue<std::string>(config, "map_filename", context),
            filepath.parent_path());
        const YAML::Node position =
            requireNode(config, "initial_drone_position", context);

        return types::SimulationConfigData{
            map_filename,
            requireValue<double>(config, "map_resolution_cm", context) * cm,
            parseMapOffset(config["map_axes_offset"]),
            parsePosition(position, context + ".initial_drone_position"),
            requireValue<double>(config, "initial_angle_deg", context) *
                horizontal_angle[deg],
        };
    } catch (const std::exception& exception) {
        throw wrapParseError(filepath, exception);
    }
}

types::SimulationCompositionData
ConfigParser::parseSimulationComposition(const std::filesystem::path& filepath) {
    try {
        const std::filesystem::path composition_file = normalizedAbsolute(filepath);
        const YAML::Node root = loadRoot(composition_file);
        const YAML::Node compositions =
            requireNode(root, "simulation_compositions", contextFor(composition_file, "root"));
        const std::string context = contextFor(composition_file, "simulation_compositions");
        const std::filesystem::path composition_dir = composition_file.parent_path();

        std::vector<std::tuple<types::SimulationConfigData, std::vector<types::MissionConfigData>>>
            simulation_mission_groups;
        const YAML::Node simulations = requireNode(compositions, "simulations", context);
        if (!simulations.IsSequence() || simulations.size() == 0) {
            throw std::runtime_error("'simulations' must be a non-empty sequence in " +
                                     context + ".");
        }

        for (const YAML::Node& simulation_entry : simulations) {
            const std::filesystem::path simulation_file = resolvePath(
                requireValue<std::string>(simulation_entry, "simulation_config", context),
                composition_dir);
            types::SimulationConfigData simulation =
                ConfigParser::parseSimulationConfig(simulation_file);

            const YAML::Node mission_files =
                requireNode(simulation_entry, "mission_configs", context + ".simulations[]");
            if (!mission_files.IsSequence() || mission_files.size() == 0) {
                throw std::runtime_error("'mission_configs' must be a non-empty sequence in " +
                                         context + ".");
            }

            std::vector<types::MissionConfigData> missions;
            for (const YAML::Node& mission_file_node : mission_files) {
                const std::filesystem::path mission_file = resolvePath(
                    mission_file_node.as<std::string>(),
                    composition_dir);
                missions.push_back(ConfigParser::parseMissionConfig(mission_file));
            }

            simulation_mission_groups.emplace_back(std::move(simulation), std::move(missions));
        }

        std::vector<types::DroneConfigData> drones;
        const YAML::Node drone_files = requireNode(compositions, "drone_configs", context);
        if (!drone_files.IsSequence() || drone_files.size() == 0) {
            throw std::runtime_error("'drone_configs' must be a non-empty sequence in " +
                                     context + ".");
        }
        for (const YAML::Node& drone_file_node : drone_files) {
            const std::filesystem::path drone_file = resolvePath(
                drone_file_node.as<std::string>(),
                composition_dir);
            drones.push_back(ConfigParser::parseDroneConfig(drone_file));
        }

        std::vector<types::LidarConfigData> lidars;
        const YAML::Node lidar_files = requireNode(compositions, "lidar_configs", context);
        if (!lidar_files.IsSequence() || lidar_files.size() == 0) {
            throw std::runtime_error("'lidar_configs' must be a non-empty sequence in " +
                                     context + ".");
        }
        for (const YAML::Node& lidar_file_node : lidar_files) {
            const std::filesystem::path lidar_file = resolvePath(
                lidar_file_node.as<std::string>(),
                composition_dir);
            lidars.push_back(ConfigParser::parseLidarConfig(lidar_file));
        }

        return types::SimulationCompositionData{
            composition_file,
            std::move(simulation_mission_groups),
            std::move(drones),
            std::move(lidars),
        };
    } catch (const std::exception& exception) {
        throw wrapParseError(filepath, exception);
    }
}

} // namespace drone_mapper
