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

} // namespace
} // namespace drone_mapper
