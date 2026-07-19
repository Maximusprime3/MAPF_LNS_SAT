#include "LNS.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

void print_usage(const char* executable) {
    std::cerr << "Usage: " << executable
              << " <map_path> <scenario_path> <num_agents> <scenario_index> <solver> [seed] [variant]" << std::endl
              << "   or: " << executable << " --config <config_file>" << std::endl
              << "  optional variant: lns-sat | initial-radius-2 | fixed-step-2 | increasing-step" << std::endl
              << "  example: ./main_clean_lns mapf-map/maze-32-32-2.map mapf-scen-even/scen-even/maze-32-32-2-even-1.scen 30 0 minisat 42 lns-sat" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
    ConfigurationResolution resolution;
    if (argc == 3 && std::string(argv[1]) == "--config") {
        resolution = load_solve_configuration_file(argv[2]);
    } else {
        std::vector<std::string> arguments;
        arguments.reserve(argc > 1 ? static_cast<std::size_t>(argc - 1) : 0);
        for (int index = 1; index < argc; ++index) {
            arguments.emplace_back(argv[index]);
        }
        resolution = resolve_positional_solve_arguments(arguments);
    }

    if (!resolution.valid()) {
        std::cerr << "LNS_RESULT status=invalid-input message="
                  << resolution.validation.message << std::endl;
        print_usage(argv[0]);
        return solve_status_exit_code(SolveStatus::InvalidInput);
    }

    std::unique_ptr<std::ofstream> log_stream;
    std::streambuf* original_cout_buf = nullptr;
    if (!resolution.config.log_output_path.empty()) {
        log_stream = std::make_unique<std::ofstream>(
            resolution.config.log_output_path);
        if (!log_stream->is_open()) {
            std::cerr << "LNS_RESULT status=invalid-input message="
                      << "Failed to open log file: "
                      << resolution.config.log_output_path << std::endl;
            return solve_status_exit_code(SolveStatus::InvalidInput);
        }
        original_cout_buf = std::cout.rdbuf(log_stream->rdbuf());
    }

    const LNSResult result = LNS(resolution.request, resolution.config);
    if (!result.solved()) {
        std::cerr << "LNS_RESULT status=" << solve_status_name(result.status)
                  << " message=" << result.message << std::endl;
        if (original_cout_buf != nullptr) {
            std::cout.flush();
            std::cout.rdbuf(original_cout_buf);
        }
        return solve_status_exit_code(result.status);
    }

    if (original_cout_buf != nullptr) {
        std::cout.flush();
        std::cout.rdbuf(original_cout_buf);
    }
    return 0;
}
