#pragma once
//simulator.h
#include <string>
//// Simulator is the main coordinator of the program and writing output files.
class Simulator {
public:
    // Constructor.
    explicit Simulator(const std::string& inputOutputPath);
    // Runs the main simulation loop, which involves creating the drone, movement driver, and lidar sensor, and coordinating their interactions with the building map and mission configuration. The method returns an integer status code indicating the result of the simulation.
    int run();

private:
    // Path to the input and output files for the simulation.
    std::string inputOutputPath;

    // Helper method to create a command that indicates the drone has finished its mission. This is used to signal the end of the simulation.
    std::string makePath(const std::string& fileName) const;
};