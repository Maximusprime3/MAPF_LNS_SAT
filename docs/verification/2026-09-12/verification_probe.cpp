#include "LNS.h"
#include "Load_LNSProblem.h"
#include "SolutionVerifier.h"
#include <chrono>
#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 7) return 64;
    std::ofstream log("solver.log");
    auto* original = std::cout.rdbuf(log.rdbuf());
    SolveRequest request{argv[1], argv[2], std::stoi(argv[3]), 0};
    SolverConfig config;
    config.seed = std::stoi(argv[4]);
    config.neighborhood_variant = *parse_neighborhood_variant(argv[6]);
    const int limit = std::stoi(argv[5]);
    if (limit > 0) config.wall_clock_limit = std::chrono::milliseconds(limit);
    config.log_level = LogLevel::Quiet;
    auto problem = load_problem(request.map_path, request.scenario_path, request.num_agents, 0);
    if (!problem) {
        std::cout.rdbuf(original);
        std::cout << "{\"status\":\"invalid-input\",\"verified\":false,\"paths\":0,\"makespan\":-1,\"solve_ms\":0}\n";
        return 65;
    }
    const auto start = std::chrono::steady_clock::now();
    auto result = LNS(request, config);
    const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    bool valid = false;
    int makespan = -1;
    if (result.solved()) {
        auto report = mapf::verify_solution(result.paths, problem->starts, problem->goals, problem->grid);
        valid = report.valid() && result.paths.size() == static_cast<std::size_t>(request.num_agents);
        makespan = report.makespan;
    }
    std::cout.rdbuf(original);
    std::cout << "{\"status\":\"" << solve_status_name(result.status)
              << "\",\"verified\":" << (valid ? "true" : "false")
              << ",\"paths\":" << result.paths.size() << ",\"makespan\":" << makespan
              << ",\"solve_ms\":" << elapsed << "}\n";
    return result.solved() && !valid ? 2 : 0;
}
