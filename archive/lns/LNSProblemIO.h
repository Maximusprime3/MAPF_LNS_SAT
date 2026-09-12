#pragma once

#include <vector>
#include <string>
#include <optional>

struct LNSProblem {
    std::vector<std::vector<char>> grid;
    std::vector<std::pair<int,int>> starts;
    std::vector<std::pair<int,int>> goals;
};

// Load map and scenario data for the specified number of agents
std::optional<LNSProblem> load_problem(const std::string& map_path,
                                      const std::string& scenario_path,
                                      int num_agents,
                                      int scenario_index);