# Minimum CMake version required to configure this project.
# We use 3.24 because the project uses modern CMake features such as FetchContent.
cmake_minimum_required(VERSION 3.24)

# Define the project name and specify that this is a C++ project.
project(drone_mapper_project LANGUAGES CXX)

# Use C++20, as required by the assignment guidelines.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Disable compiler-specific C++ extensions.
# This makes the build closer to standard C++20.
set(CMAKE_CXX_EXTENSIONS OFF)

# Generate compile_commands.json.
# This is useful for VS Code, clangd, and debugging the exact compile commands.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# FetchContent lets CMake download and build dependencies automatically.
include(FetchContent)

# fmt is fetched because mp-units may need it when std::format is unavailable
# or incomplete in the compiler/STL implementation.
FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 10.2.1
    OVERRIDE_FIND_PACKAGE
)
# Download and make fmt available to this project.
FetchContent_MakeAvailable(fmt)


# mp-units configuration.
# We disable modules and installation because we only need to build and link it
# as part of this project.
set(MP_UNITS_BUILD_CXX_MODULES OFF CACHE BOOL "" FORCE)
set(MP_UNITS_BUILD_INSTALL OFF CACHE BOOL "" FORCE)
set(MP_UNITS_API_CONTRACTS NONE CACHE STRING "" FORCE)

# Fetch the mp-units library used for strong typed units such as cm and degrees.
FetchContent_Declare(
    mp-units
    GIT_REPOSITORY https://github.com/mpusz/mp-units.git
    GIT_TAG v2.5.0
    SOURCE_SUBDIR src
)

# Download and make mp-units available to this project.
FetchContent_MakeAvailable(mp-units)

# Define the final executable produced by the project.
# All source files needed to build drone_mapper are listed here.
add_executable(drone_mapper
    src/main.cpp
    src/GridMap.cpp
    src/GroundTruthMap.cpp
    src/SparseBuildingMap.cpp
    src/MockPositionSensor.cpp
    src/ConfigParser.cpp
    src/MapFileWriter.cpp
    src/ScoreCalculator.cpp
    src/simulator.cpp
    src/MockMovementDriver.cpp
    src/MockLidarSensor.cpp
    src/drone.cpp
)

# Add the include directory so source files can include project headers
# such as "Types.h", "DroneConfig.h", etc.
target_include_directories(drone_mapper PRIVATE include)

# Compile our project with the flags required by the assignment:
# -Wall      enables many common warnings.
# -Wextra    enables additional warnings.
# -Werror    treats warnings as errors.
# -pedantic  enforces stricter ISO C++ compliance.
target_compile_options(drone_mapper PRIVATE
    -Wall
    -Wextra
    -Werror
    -pedantic
)

# Link the executable with the external libraries it needs.
# mp-units is used for typed units.
# fmt is needed by mp-units in this setup.
target_link_libraries(drone_mapper PRIVATE
    mp-units::mp-units
    fmt::fmt
)
