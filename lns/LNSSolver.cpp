#include "../SATSolverManager.h"
#include <vector>
#include <string>
#include <iostream>
#include <optional>

struct LNSProblem {
    std::vector<std::vector<char>> grid;
    std::vector<std::pair<int,int>> starts;
    std::vector<std::pair<int,int>> goals;
};

// Loads the map and the requested scenario entry, extracting starts/goals for num_agents
static std::optional<LNSProblem> load_problem(const std::string& map_path,
                                              const std::string& scenario_path,
                                              int num_agents,
                                              int scenario_index /*0-based*/) {
    LNSProblem problem;

    // Load map
    problem.grid = SATSolverManager::load_map(map_path);
    if (problem.grid.empty()) {
        std::cerr << "[LNS] Failed to load map from: " << map_path << std::endl;
        return std::nullopt;
    }

    // Load scenario entries and build starts/goals sets
    auto entries = SATSolverManager::create_dataframe_from_file(scenario_path);
    if (entries.empty()) {
        std::cerr << "[LNS] No entries found in scenario: " << scenario_path << std::endl;
        return std::nullopt;
    }
    auto sets = SATSolverManager::create_starts_and_goals(entries, num_agents);
    if (sets.empty()) {
        std::cerr << "[LNS] No start/goal sets could be formed from scenario: " << scenario_path << std::endl;
        return std::nullopt;
    }
    if (scenario_index < 0 || scenario_index >= static_cast<int>(sets.size())) {
        std::cerr << "[LNS] scenario_index out of range: " << scenario_index
                  << ", available sets: " << sets.size() << std::endl;
        return std::nullopt;
    }

    problem.starts = sets[scenario_index].first;
    problem.goals = sets[scenario_index].second;

    return problem;
}

// Entry point for crude LNS (skeleton). Future steps will be filled in iteratively.
int run_crude_lns(const std::string& map_path,
                  const std::string& scenario_path,
                  int num_agents,
                  int scenario_index /*0-based*/,
                  bool use_minisat /*else ProbSAT*/,
                  int seed) {
    // Step 1: Load selected scenario and map (grid, starts, goals)
    auto problem_opt = load_problem(map_path, scenario_path, num_agents, scenario_index);
    if (!problem_opt.has_value()) {
        return 1; // failed to load
    }
    const auto& problem = problem_opt.value();

    std::cout << "[LNS] Loaded map " << problem.grid.size() << "x"
              << (problem.grid.empty() ? 0 : (int)problem.grid[0].size())
              << ", agents: " << problem.starts.size() << std::endl;

    // Print full map
    std::cout << "[LNS] Map:" << std::endl;
    for (const auto& row : problem.grid) {
        for (char c : row) std::cout << c;
        std::cout << '\n';
    }

    // Print starts and goals
    std::cout << "[LNS] Agents (start -> goal):" << std::endl;
    for (size_t i = 0; i < problem.starts.size(); ++i) {
        const auto& s = problem.starts[i];
        const auto& g = problem.goals[i];
        std::cout << "  Agent " << i << ": (" << s.first << "," << s.second << ") -> ("
                  << g.first << "," << g.second << ")" << std::endl;
    }

    // Remaining steps will be implemented next (todos):
    // - Construct MDDs for all agents on the full map
    // - Sample full solution paths from MDDs to get a faulty solution
    // - Detect conflicts (vertex and edge) in sampled solution
    // - Define conflict zones/windows for each detected conflict
    // - For each zone, create masked map and local agent subsets
    // - Construct local MDDs and CNF for each conflict subproblem
    // - Solve each conflict subproblem with SAT/ProbSAT; expand window on failure
    // - Integrate local solutions into the global solution paths
    // - Re-check for remaining conflicts and iterate until none or limit

    return 0;
}
