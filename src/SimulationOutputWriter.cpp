#include <drone_mapper/SimulationOutputWriter.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_mapper {

namespace {

// Raw file-name layout recovered from the composition YAML, kept in the exact
// order the manager iterates so it can be zipped with report.runs.
struct CompositionLayout {
    std::vector<std::pair<std::string, std::vector<std::string>>> simulations;
    std::vector<std::string> drones;
    std::vector<std::string> lidars;
};

[[nodiscard]] std::vector<std::string> stringSequence(const YAML::Node& node) {
    std::vector<std::string> values;
    if (!node || !node.IsSequence()) {
        return values;
    }
    for (const YAML::Node& entry : node) {
        values.push_back(entry.as<std::string>());
    }
    return values;
}

[[nodiscard]] CompositionLayout readLayout(const std::filesystem::path& composition_file) {
    CompositionLayout layout;
    const YAML::Node root = YAML::LoadFile(composition_file.string());
    const YAML::Node compositions = root["simulation_compositions"];
    if (!compositions) {
        throw std::runtime_error("Missing 'simulation_compositions' in " +
                                 composition_file.string());
    }

    const YAML::Node simulations = compositions["simulations"];
    if (simulations && simulations.IsSequence()) {
        for (const YAML::Node& simulation : simulations) {
            std::string simulation_config = simulation["simulation_config"]
                                                ? simulation["simulation_config"].as<std::string>()
                                                : std::string{};
            layout.simulations.emplace_back(std::move(simulation_config),
                                            stringSequence(simulation["mission_configs"]));
        }
    }

    layout.drones = stringSequence(compositions["drone_configs"]);
    layout.lidars = stringSequence(compositions["lidar_configs"]);
    return layout;
}

[[nodiscard]] const char* missionStatusString(types::MissionRunStatus status) {
    switch (status) {
    case types::MissionRunStatus::Completed:
        return "completed";
    case types::MissionRunStatus::MaxSteps:
        return "max_steps";
    case types::MissionRunStatus::Error:
        return "error";
    }
    return "error";
}

[[nodiscard]] const char* resolutionStatusString(types::ResolutionRequestStatus status) {
    switch (status) {
    case types::ResolutionRequestStatus::Accepted:
        return "ACCEPTED";
    case types::ResolutionRequestStatus::Ignored:
        return "IGNORED";
    case types::ResolutionRequestStatus::IgnoredTooSmall:
        return "IGNORED_TOO_SMALL";
    }
    return "IGNORED";
}

[[nodiscard]] bool isErrorRun(const types::SimulationResult& result) {
    if (result.mission_score < 0.0) {
        return true;
    }
    if (result.mission_results.empty()) {
        return true;
    }
    return result.mission_results.front().status == types::MissionRunStatus::Error;
}

struct Summary {
    std::size_t total_runs = 0;
    std::size_t scored_runs = 0;
    std::size_t error_runs = 0;
    double average_score = 0.0;
    double min_score = 0.0;
    double max_score = 0.0;
};

[[nodiscard]] Summary summarize(const std::vector<types::SimulationResult>& runs) {
    Summary summary;
    summary.total_runs = runs.size();

    double sum = 0.0;
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();

    for (const types::SimulationResult& run : runs) {
        if (isErrorRun(run)) {
            ++summary.error_runs;
            continue;
        }
        ++summary.scored_runs;
        sum += run.mission_score;
        min_score = std::min(min_score, run.mission_score);
        max_score = std::max(max_score, run.mission_score);
    }

    if (summary.scored_runs > 0) {
        summary.average_score = sum / static_cast<double>(summary.scored_runs);
        summary.min_score = min_score;
        summary.max_score = max_score;
    }
    return summary;
}

void emitRun(YAML::Emitter& out,
             const std::string& drone_config,
             const std::string& lidar_config,
             const types::SimulationResult& result) {
    const types::MissionRunResult* mission =
        result.mission_results.empty() ? nullptr : &result.mission_results.front();

    out << YAML::BeginMap;
    out << YAML::Key << "drone_config" << YAML::Value << drone_config;
    out << YAML::Key << "lidar_config" << YAML::Value << lidar_config;
    out << YAML::Key << "status" << YAML::Value
        << (mission ? missionStatusString(mission->status) : "error");
    out << YAML::Key << "steps" << YAML::Value
        << (mission ? mission->steps : static_cast<std::size_t>(0));
    out << YAML::Key << "score" << YAML::Value << result.mission_score;

    if (isErrorRun(result)) {
        std::string code = "UNKNOWN_ERROR";
        if (mission != nullptr && !mission->errors.empty()) {
            code = mission->errors.front().code;
        }
        out << YAML::Key << "error_ref" << YAML::Value << YAML::BeginMap
            << YAML::Key << "code" << YAML::Value << code << YAML::EndMap;
    }

    out << YAML::EndMap;
}

} // namespace

void SimulationOutputWriter::write(const std::filesystem::path& composition_file,
                                   const types::SimulationManagerReport& report,
                                   const std::filesystem::path& output_path) {
    const CompositionLayout layout = readLayout(composition_file);
    const Summary summary = summarize(report.runs);
    const std::size_t runs_per_mission = layout.drones.size() * layout.lidars.size();

    YAML::Emitter out;
    out.SetDoublePrecision(4);
    out.SetFloatPrecision(4);

    out << YAML::BeginMap;
    out << YAML::Key << "score_report" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "composition_file" << YAML::Value
        << composition_file.filename().string();
    out << YAML::Key << "generated_at_utc" << YAML::Value << report.generated_at_utc;
    out << YAML::Key << "metric" << YAML::Value << report.metric;
    out << YAML::Key << "score_range" << YAML::Value << YAML::BeginMap
        << YAML::Key << "min" << YAML::Value << std::get<0>(report.score_range)
        << YAML::Key << "max" << YAML::Value << std::get<1>(report.score_range) << YAML::EndMap;
    out << YAML::Key << "error_score" << YAML::Value << report.error_score;

    out << YAML::Key << "summary" << YAML::Value << YAML::BeginMap
        << YAML::Key << "total_runs" << YAML::Value << summary.total_runs
        << YAML::Key << "scored_runs" << YAML::Value << summary.scored_runs
        << YAML::Key << "error_runs" << YAML::Value << summary.error_runs
        << YAML::Key << "average_score" << YAML::Value << summary.average_score
        << YAML::Key << "min_score" << YAML::Value << summary.min_score
        << YAML::Key << "max_score" << YAML::Value << summary.max_score << YAML::EndMap;

    out << YAML::Key << "simulations" << YAML::Value << YAML::BeginSeq;

    std::size_t run_index = 0;
    for (const auto& [simulation_config, mission_configs] : layout.simulations) {
        out << YAML::BeginMap;
        out << YAML::Key << "simulation_config" << YAML::Value << simulation_config;
        out << YAML::Key << "missions" << YAML::Value << YAML::BeginSeq;

        for (const std::string& mission_config : mission_configs) {
            const std::size_t mission_first = run_index;

            // resolution metadata is shared by all runs of a mission; take it
            // from the first run that actually produced an output map.
            double resolution_cm = 0.0;
            types::ResolutionRequestStatus resolution_status =
                types::ResolutionRequestStatus::Ignored;
            for (std::size_t offset = 0;
                 offset < runs_per_mission && (mission_first + offset) < report.runs.size();
                 ++offset) {
                const types::SimulationResult& candidate = report.runs[mission_first + offset];
                if (offset == 0) {
                    resolution_status = candidate.resolution_request_status;
                }
                const double candidate_resolution =
                    candidate.output_map_config.resolution.force_numerical_value_in(cm);
                if (resolution_cm <= 0.0 && candidate_resolution > 0.0) {
                    resolution_cm = candidate_resolution;
                    resolution_status = candidate.resolution_request_status;
                }
            }

            out << YAML::BeginMap;
            out << YAML::Key << "mission_config" << YAML::Value << mission_config;
            out << YAML::Key << "resolution_cm" << YAML::Value << resolution_cm;
            out << YAML::Key << "resolution_request_status" << YAML::Value
                << resolutionStatusString(resolution_status);
            out << YAML::Key << "runs" << YAML::Value << YAML::BeginSeq;

            for (const std::string& drone_config : layout.drones) {
                for (const std::string& lidar_config : layout.lidars) {
                    if (run_index < report.runs.size()) {
                        emitRun(out, drone_config, lidar_config, report.runs[run_index]);
                    }
                    ++run_index;
                }
            }

            out << YAML::EndSeq; // runs
            out << YAML::EndMap;  // mission
        }

        out << YAML::EndSeq; // missions
        out << YAML::EndMap;  // simulation
    }

    out << YAML::EndSeq; // simulations
    out << YAML::EndMap; // score_report
    out << YAML::EndMap; // root

    std::filesystem::create_directories(output_path);
    const std::filesystem::path output_file = output_path / "simulation_output.yaml";
    std::ofstream stream{output_file, std::ios::trunc};
    if (!stream) {
        throw std::runtime_error("Failed to open simulation output file: " + output_file.string());
    }
    stream << out.c_str() << '\n';
}

} // namespace drone_mapper
