cmake_minimum_required(VERSION 3.15)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)


find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED) # new!

add_library(drone_mapper
    src/ConfigParser.cpp
    src/DroneControlImpl.cpp
    src/MappingAlgorithmImpl.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/Map3DImpl.cpp
    src/NpyMapIO.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/ScanResultToVoxels.cpp
    src/SimulationManager.cpp
    src/SimulationOutputWriter.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_compile_options(drone_mapper
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
        yaml-cpp::yaml-cpp
)

add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)
#Compile the maps_comparison util
add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

# Compile the yaml example file
add_executable(example_yml cpp_yaml_example/main.cpp)
target_link_libraries(example_yml PRIVATE yaml-cpp::yaml-cpp)

# ---------------------------------------------------------------------------
# Tests: component tests (tests/components) + integration tests (tests/integration).
# Run all:           ./drone_mapper_simulation_test
# Run a component:   ./drone_mapper_simulation_test --gtest_filter=MapsComparison.*
# Run integration:   ./drone_mapper_simulation_test --gtest_filter=Integration.*
# ---------------------------------------------------------------------------
find_package(GTest CONFIG REQUIRED)

add_executable(drone_mapper_simulation_test
    tests/components/SimulationManagerTests.cpp
    tests/components/SimulationRunTests.cpp
    tests/components/MissionControlTests.cpp
    tests/components/DroneControlTests.cpp
    tests/components/MappingAlgorithmTests.cpp
    tests/components/MockLidarTests.cpp
    tests/components/MapsComparisonTests.cpp
    tests/integration/IntegrationTests.cpp
    tests/components/ConfigParserTests.cpp
)
target_include_directories(drone_mapper_simulation_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
)
target_link_libraries(drone_mapper_simulation_test
    PRIVATE
        drone_mapper
        GTest::gtest
        GTest::gmock
        GTest::gtest_main
)
