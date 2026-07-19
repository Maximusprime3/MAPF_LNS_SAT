#include "Load_LNSProblem.h"
#include "../SATSolverManager.h"
#include <iostream>

std::optional<LNSProblem> load_problem(const std::string& map_path,
                                       const std::string& scenario_path,
                                       int num_agents,
                                       int scenario_index) {
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

    auto is_walkable_position = [&](const std::pair<int, int>& position) {
        const int row = position.first;
        const int column = position.second;
        if (row < 0 || row >= static_cast<int>(problem.grid.size()) ||
            column < 0 ||
            column >= static_cast<int>(problem.grid[row].size())) {
            return false;
        }
        const char cell = problem.grid[row][column];
        return cell == '.' || cell == 'G';
    };
    for (std::size_t agent_id = 0; agent_id < problem.starts.size(); ++agent_id) {
        if (!is_walkable_position(problem.starts[agent_id]) ||
            !is_walkable_position(problem.goals[agent_id])) {
            std::cerr << "[LNS] Agent " << agent_id
                      << " has a blocked or out-of-grid start/goal" << std::endl;
            return std::nullopt;
        }
    }

    return problem;
}
