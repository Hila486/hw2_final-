#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>

namespace drone_mapper {

// Writes the Assignment-2 score_report YAML (simulation_output.yaml) into the
// output path. The manager report is a flat list of runs in Cartesian order;
// the composition file is re-read to recover the per-config file names (in that
// same order) and to emit the required simulations -> missions -> runs
// hierarchy.
class SimulationOutputWriter {
public:
    static void write(const std::filesystem::path& composition_file,
                      const types::SimulationManagerReport& report,
                      const std::filesystem::path& output_path);
};

} // namespace drone_mapper
