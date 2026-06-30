#include <drone_mapper/ConfigParser.h>
#include <drone_mapper/SimulationManager.h> // include the class that control running the simulations
#include <drone_mapper/SimulationOutputWriter.h> // writes simulation_output.yaml
#include <drone_mapper/SimulationRunFactoryImpl.h> // include the class that create the simulation run

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>

namespace {

std::filesystem::path resolveInputPath(const std::filesystem::path& path) {
    if (path.is_absolute()) {
        return path.lexically_normal();
    }

    return std::filesystem::absolute(path).lexically_normal();
}

std::filesystem::path compositionPath(int argc, char** argv) {
    return resolveInputPath(
        (argc >= 2) ? std::filesystem::path{argv[1]} : std::filesystem::path{"simulation.yaml"});
}

std::filesystem::path outputPath(int argc, char** argv) {
    if (argc >= 3) {
        return resolveInputPath(std::filesystem::path{argv[2]});
    }

    return std::filesystem::current_path();
}

} // namespace

int main(int argc, char** argv) { // program entry point, argc is the number of arguments, argv is the arguments
    if (argc > 3) {
        std::cerr << "Usage: ./drone_mapper_simulation [<simulation.yaml>] [<output_path>]\n";
        return 1;
    }

    try {
        const std::filesystem::path composition_file = compositionPath(argc, argv);
        const std::filesystem::path output_path = outputPath(argc, argv);

        const drone_mapper::types::SimulationCompositionData composition =
            drone_mapper::ConfigParser::parseSimulationComposition(composition_file);

        auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
        drone_mapper::SimulationManager simulation{std::move(run_factory)}; // create the simulation manager and give it ownership of the run factory

        const drone_mapper::types::SimulationManagerReport report =
            simulation.run(composition, output_path);

        // Writing the report must not discard completed runs: a failure here is
        // logged but does not abort the program.
        try {
            drone_mapper::SimulationOutputWriter::write(composition_file, report, output_path);
        } catch (const std::exception& exception) {
            std::cerr << "Failed to write simulation_output.yaml: " << exception.what() << '\n';
        }

        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Simulation failed: " << exception.what() << '\n';
        return 1;
    }
}
//save
