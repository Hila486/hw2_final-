#include <drone_mapper/ConfigParser.h>

#include "support/TestHelpers.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace drone_mapper {
namespace {

void writeTextFile(const std::filesystem::path& path, const std::string& text) {
    std::ofstream out(path);
    ASSERT_TRUE(out.good()) << "Failed to open " << path;
    out << text;
}

TEST(ConfigParser, MissingInitialAngleDefaultsToZero) {
    const auto dir = test::makeTempDir("config_parser_missing_angle");
    const auto path = dir / "simulation.yaml";

    writeTextFile(path, R"YAML(
simulation_config:
  map_filename: dummy.npy
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: 100
    y_cm: 200
    height_cm: 30
)YAML");

    const types::SimulationConfigData config =
        ConfigParser::parseSimulationConfig(path);

    EXPECT_EQ(config.initial_angle, test::Hdeg(0));
    EXPECT_EQ(config.map_resolution, test::L(10));
    EXPECT_EQ(config.initial_drone_position.x, test::X(100));
    EXPECT_EQ(config.initial_drone_position.y, test::Y(200));
    EXPECT_EQ(config.initial_drone_position.z, test::Z(30));
}

TEST(ConfigParser, ExplicitInitialAngleIsParsed) {
    const auto dir = test::makeTempDir("config_parser_explicit_angle");
    const auto path = dir / "simulation.yaml";

    writeTextFile(path, R"YAML(
simulation_config:
  map_filename: dummy.npy
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: 100
    y_cm: 200
    height_cm: 30
  initial_angle_deg: 90
)YAML");

    const types::SimulationConfigData config =
        ConfigParser::parseSimulationConfig(path);

    EXPECT_EQ(config.initial_angle, test::Hdeg(90));
}

TEST(ConfigParser, MissingOutputMappingResolutionFactorDefaultsToOne) {
    const auto dir = test::makeTempDir("config_parser_missing_resolution_factor");
    const auto path = dir / "mission.yaml";

    writeTextFile(path, R"YAML(
mission_config:
  max_steps: 2400
  boundaries:
    x_boundary:
      min_cm: 0
      max_cm: 100
    y_boundary:
      min_cm: 0
      max_cm: 100
    height_boundary:
      min_cm: 0
      max_cm: 50
  gps_resolution_cm: 10
)YAML");

    const types::MissionConfigData config =
        ConfigParser::parseMissionConfig(path);

    EXPECT_EQ(config.max_steps, 2400u);
    EXPECT_EQ(config.gps_resolution, test::L(10));
    EXPECT_DOUBLE_EQ(config.output_mapping_resolution_factor, 1.0);
}

TEST(ConfigParser, ExplicitOutputMappingResolutionFactorIsParsed) {
    const auto dir = test::makeTempDir("config_parser_explicit_resolution_factor");
    const auto path = dir / "mission.yaml";

    writeTextFile(path, R"YAML(
mission_config:
  max_steps: 2400
  boundaries:
    x_boundary:
      min_cm: 0
      max_cm: 100
    y_boundary:
      min_cm: 0
      max_cm: 100
    height_boundary:
      min_cm: 0
      max_cm: 50
  gps_resolution_cm: 10
  output_mapping_resolution_factor: 2
)YAML");

    const types::MissionConfigData config =
        ConfigParser::parseMissionConfig(path);

    EXPECT_DOUBLE_EQ(config.output_mapping_resolution_factor, 2.0);
}

TEST(ConfigParser, MissingMapAxesOffsetDefaultsToZero) {
    const auto dir = test::makeTempDir("config_parser_missing_map_offset");
    const auto path = dir / "simulation.yaml";

    writeTextFile(path, R"YAML(
simulation_config:
  map_filename: dummy.npy
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: 10
    y_cm: 20
    height_cm: 30
  initial_angle_deg: 0
)YAML");

    const types::SimulationConfigData config =
        ConfigParser::parseSimulationConfig(path);

    EXPECT_EQ(config.map_offset.x, test::X(0));
    EXPECT_EQ(config.map_offset.y, test::Y(0));
    EXPECT_EQ(config.map_offset.z, test::Z(0));
}

TEST(ConfigParser, MissingRequiredSimulationMapFilenameThrows) {
    const auto dir = test::makeTempDir("config_parser_missing_map_filename");
    const auto path = dir / "simulation.yaml";

    writeTextFile(path, R"YAML(
simulation_config:
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: 100
    y_cm: 200
    height_cm: 30
  initial_angle_deg: 0
)YAML");

    EXPECT_THROW((void)ConfigParser::parseSimulationConfig(path), std::exception);
}

TEST(ConfigParser, MissingRequiredSimulationInitialPositionThrows) {
    const auto dir = test::makeTempDir("config_parser_missing_initial_position");
    const auto path = dir / "simulation.yaml";

    writeTextFile(path, R"YAML(
simulation_config:
  map_filename: dummy.npy
  map_resolution_cm: 10
  initial_angle_deg: 0
)YAML");

    EXPECT_THROW((void)ConfigParser::parseSimulationConfig(path), std::exception);
}

TEST(ConfigParser, MissingRequiredDroneDimensionsThrows) {
    const auto dir = test::makeTempDir("config_parser_missing_drone_dimensions");
    const auto path = dir / "drone.yaml";

    writeTextFile(path, R"YAML(
drone_config:
  max_rotate_deg: 45
  max_advance_cm: 50
  max_elevate_cm: 40
)YAML");

    EXPECT_THROW((void)ConfigParser::parseDroneConfig(path), std::exception);
}

TEST(ConfigParser, MissingRequiredMissionBoundariesThrows) {
    const auto dir = test::makeTempDir("config_parser_missing_mission_boundaries");
    const auto path = dir / "mission.yaml";

    writeTextFile(path, R"YAML(
mission_config:
  max_steps: 2400
  gps_resolution_cm: 10
  output_mapping_resolution_factor: 1
)YAML");

    EXPECT_THROW((void)ConfigParser::parseMissionConfig(path), std::exception);
}

TEST(ConfigParser, MissingRequiredMissionGpsResolutionThrows) {
    const auto dir = test::makeTempDir("config_parser_missing_gps_resolution");
    const auto path = dir / "mission.yaml";

    writeTextFile(path, R"YAML(
mission_config:
  max_steps: 2400
  boundaries:
    x_boundary:
      min_cm: 0
      max_cm: 100
    y_boundary:
      min_cm: 0
      max_cm: 100
    height_boundary:
      min_cm: 0
      max_cm: 50
)YAML");

    EXPECT_THROW((void)ConfigParser::parseMissionConfig(path), std::exception);
}

TEST(ConfigParser, MissingRequiredLidarZMaxThrows) {
    const auto dir = test::makeTempDir("config_parser_missing_lidar_zmax");
    const auto path = dir / "lidar.yaml";

    writeTextFile(path, R"YAML(
lidar_config:
  z_min_cm: 20
  d_cm: 2.5
  fov_circles: 5
)YAML");

    EXPECT_THROW((void)ConfigParser::parseLidarConfig(path), std::exception);
}

TEST(ConfigParser, MissingCompositionDroneConfigsThrows) {
    const auto dir = test::makeTempDir("config_parser_missing_composition_drones");

    writeTextFile(dir / "simulation.yaml", R"YAML(
simulation_config:
  map_filename: dummy.npy
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: 10
    y_cm: 20
    height_cm: 30
  initial_angle_deg: 0
)YAML");

    writeTextFile(dir / "mission.yaml", R"YAML(
mission_config:
  max_steps: 100
  boundaries:
    x_boundary:
      min_cm: 0
      max_cm: 100
    y_boundary:
      min_cm: 0
      max_cm: 100
    height_boundary:
      min_cm: 0
      max_cm: 50
  gps_resolution_cm: 10
)YAML");

    const auto composition = dir / "composition.yaml";
    writeTextFile(composition, R"YAML(
simulation_compositions:
  simulations:
    - simulation_config: simulation.yaml
      mission_configs:
        - mission.yaml
  lidar_configs:
    - lidar.yaml
)YAML");

    EXPECT_THROW((void)ConfigParser::parseSimulationComposition(composition), std::exception);
}

TEST(ConfigParser, InvalidMissionMaxStepsTypeThrows) {
    const auto dir = test::makeTempDir("config_parser_invalid_max_steps_type");
    const auto path = dir / "mission.yaml";

    writeTextFile(path, R"YAML(
mission_config:
  max_steps: not_a_number
  boundaries:
    x_boundary:
      min_cm: 0
      max_cm: 100
    y_boundary:
      min_cm: 0
      max_cm: 100
    height_boundary:
      min_cm: 0
      max_cm: 50
  gps_resolution_cm: 10
)YAML");

    EXPECT_THROW((void)ConfigParser::parseMissionConfig(path), std::exception);
}

TEST(ConfigParser, EmptyCompositionSimulationsThrows) {
    const auto dir = test::makeTempDir("config_parser_empty_simulations");
    const auto composition = dir / "composition.yaml";

    writeTextFile(composition, R"YAML(
simulation_compositions:
  simulations: []
  drone_configs:
    - drone.yaml
  lidar_configs:
    - lidar.yaml
)YAML");

    EXPECT_THROW((void)ConfigParser::parseSimulationComposition(composition), std::exception);
}

TEST(ConfigParser, EmptyCompositionLidarConfigsThrows) {
    const auto dir = test::makeTempDir("config_parser_empty_lidars");
    const auto composition = dir / "composition.yaml";

    writeTextFile(composition, R"YAML(
simulation_compositions:
  simulations:
    - simulation_config: simulation.yaml
      mission_configs:
        - mission.yaml
  drone_configs:
    - drone.yaml
  lidar_configs: []
)YAML");

    EXPECT_THROW((void)ConfigParser::parseSimulationComposition(composition), std::exception);
}

TEST(ConfigParser, CompositionRelativePathsResolveAgainstCompositionDirectory) {
    const auto dir = test::makeTempDir("config_parser_relative_paths");

    writeTextFile(dir / "simulation.yaml", R"YAML(
simulation_config:
  map_filename: dummy.npy
  map_resolution_cm: 10
  initial_drone_position:
    x_cm: 10
    y_cm: 20
    height_cm: 30
  initial_angle_deg: 0
)YAML");

    writeTextFile(dir / "mission.yaml", R"YAML(
mission_config:
  max_steps: 100
  boundaries:
    x_boundary:
      min_cm: 0
      max_cm: 100
    y_boundary:
      min_cm: 0
      max_cm: 100
    height_boundary:
      min_cm: 0
      max_cm: 50
  gps_resolution_cm: 10
)YAML");

    writeTextFile(dir / "drone.yaml", R"YAML(
drone_config:
  dimensions_cm: 10
  max_rotate_deg: 45
  max_advance_cm: 50
  max_elevate_cm: 40
)YAML");

    writeTextFile(dir / "lidar.yaml", R"YAML(
lidar_config:
  z_min_cm: 20
  z_max_cm: 120
  d_cm: 2.5
  fov_circles: 5
)YAML");

    const auto composition = dir / "composition.yaml";
    writeTextFile(composition, R"YAML(
simulation_compositions:
  simulations:
    - simulation_config: simulation.yaml
      mission_configs:
        - mission.yaml
  drone_configs:
    - drone.yaml
  lidar_configs:
    - lidar.yaml
)YAML");

    EXPECT_NO_THROW((void)ConfigParser::parseSimulationComposition(composition));
}

} // namespace
} // namespace drone_mapper
