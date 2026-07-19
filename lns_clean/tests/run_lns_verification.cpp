#include "../LNS.h"
#include "../Load_LNSProblem.h"
#include "../SolutionVerifier.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

void print_usage(const char* executable) {
    std::cerr << "Usage: " << executable
              << " <map> <scenario> <num_agents> <scenario_index> <seed> [variant]" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 6 && argc != 7) {
        print_usage(argv[0]);
        return 64;
    }

    const std::string map_path = argv[1];
    const std::string scenario_path = argv[2];
    const int num_agents = std::atoi(argv[3]);
    const int scenario_index = std::atoi(argv[4]);
    const int seed = std::atoi(argv[5]);
    const std::string variant_name = (argc == 7) ? argv[6] : "lns-sat";

    if (num_agents <= 0 || scenario_index < 0) {
        std::cerr << "Invalid agent count or scenario index" << std::endl;
        return 64;
    }

    const auto variant = parse_neighborhood_variant(variant_name);
    if (!variant.has_value()) {
        std::cerr << "Unknown neighborhood variant: " << variant_name << std::endl;
        return 64;
    }

    // Load the expected starts/goals independently of LNS so verification is
    // performed against the benchmark instance, not solver-owned metadata.
    const auto problem = load_problem(
        map_path, scenario_path, num_agents, scenario_index);
    if (!problem.has_value()) {
        std::cerr << "Could not load benchmark instance" << std::endl;
        return 65;
    }

    SolverConfig config;
    config.seed = seed;
    config.neighborhood_variant = *variant;
    const auto solve_result = LNS(
        SolveRequest{map_path, scenario_path, num_agents, scenario_index},
        config);
    if (!solve_result.solved()) {
        std::cerr << "VERIFY_RESULT seed=" << seed
                  << " status=" << solve_status_name(solve_result.status)
                  << " message=" << solve_result.message << std::endl;
        return solve_status_exit_code(solve_result.status);
    }

    // Validate the value returned by LNS, independent of its internal helper.
    // This catches the current bug where LNS can log a verification failure
    // but still return a non-empty path map.
    const auto report = mapf::verify_solution(
        solve_result.paths, problem->starts, problem->goals, problem->grid);
    if (!report.valid()) {
        for (const auto& issue : report.issues) {
            std::cerr << "VERIFY_ISSUE seed=" << seed << " code="
                      << mapf::verification_issue_code_name(issue.code)
                      << " message=" << issue.message << std::endl;
        }
        std::cerr << "VERIFY_RESULT seed=" << seed << " status=invalid" << std::endl;
        return 2;
    }

    std::cout << "VERIFY_RESULT seed=" << seed
              << " variant=" << neighborhood_variant_name(*variant)
              << " status=valid agents=" << solve_result.paths.size()
              << " makespan=" << report.makespan << std::endl;
    return 0;
}
