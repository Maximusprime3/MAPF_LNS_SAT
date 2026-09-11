#include "Load_LNSProblem.h"
#include "Grid.h"
#include "../SATSolverManager.h"
#include <iostream>
#include <algorithm>

std::optional<LNSProblem> load_problem(const std::string& map_path,
                                       const std::string& scenario_path,
                                       int num_agents,
                                       int scenario_index) {
    LNSProblem problem;
    if (num_agents <= 0 || scenario_index < 0) return std::nullopt;

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

    // scenario_index selects a group of num_agents rows, not a row offset.
    const std::size_t first_row = static_cast<std::size_t>(scenario_index) * num_agents;
    const std::size_t last_row = std::min(entries.size(), first_row + num_agents);
    for (std::size_t row = first_row; row < last_row; ++row) {
        if (entries[row].map_height != static_cast<int>(problem.grid.size()) ||
            entries[row].map_width != static_cast<int>(problem.grid.front().size())) {
            std::cerr << "[LNS] Scenario dimensions do not match the map" << std::endl;
            return std::nullopt;
        }
    }
    problem.starts = sets[scenario_index].first;
    problem.goals = sets[scenario_index].second;
    if (problem.starts.size() != static_cast<std::size_t>(num_agents) ||
        problem.goals.size() != static_cast<std::size_t>(num_agents)) {
        std::cerr << "[LNS] Selected scenario group contains fewer agents than requested" << std::endl;
        return std::nullopt;
    }

    for (std::size_t agent_id = 0; agent_id < problem.starts.size(); ++agent_id) {
        if (!mapf::is_walkable_position(problem.grid, problem.starts[agent_id]) ||
            !mapf::is_walkable_position(problem.grid, problem.goals[agent_id])) {
            std::cerr << "[LNS] Agent " << agent_id
                      << " has a blocked or out-of-grid start/goal" << std::endl;
            return std::nullopt;
        }
    }

    return problem;
}
