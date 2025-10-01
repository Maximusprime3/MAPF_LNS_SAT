#include "VerificationHelpers.h"

#include <iostream>



bool verify_path_consistency(
    const std::vector<std::pair<int, int>>& path,
    const std::vector<std::vector<char>>& map) {
    if (path.empty()) {
        return false;
    }

    auto last_pos = path.front();
    for (size_t t = 0; t < path.size(); ++t) {
        const auto& pos = path[t];
        // Check bounds
        if (pos.first < 0 || pos.first >= static_cast<int>(map.size()) ||
            pos.second < 0 || (map.empty() ? true : pos.second >= static_cast<int>(map[0].size()))) {
            std::cout << "[LNS] ERROR: Path is out of bounds at timestep " << t << std::endl;
            return false;
        }
        // Check walkability
        if (map[pos.first][pos.second] != '.' && map[pos.first][pos.second] != 'G') {
            std::cout << "map[" << pos.first << "][" << pos.second << "]: " << map[pos.first][pos.second] << std::endl;
            std::cout << "[LNS] ERROR: Path is not walkable at timestep " << t << std::endl;
            return false;
        }
        // Check adjacency (including waits)
        const bool same_row = pos.first == last_pos.first;
        const bool same_col = pos.second == last_pos.second;
        const bool neighbor_vertical = (pos.first == last_pos.first + 1 || pos.first == last_pos.first - 1) && same_col;
        const bool neighbor_horizontal = (pos.second == last_pos.second + 1 || pos.second == last_pos.second - 1) && same_row;
        if (!(same_row && same_col) && !neighbor_vertical && !neighbor_horizontal) {
            std::cout << "[LNS] ERROR: Path is not consistent at timestep " << t
                      << " " << pos.first << "," << pos.second
                      << " is not neighbor of " << last_pos.first << "," << last_pos.second << std::endl;
            std::cout << "path: ";
            for (const auto& step : path) {
                std::cout << "(" << step.first << "," << step.second << ") ";
            }
            std::cout << std::endl;
            return false;
        }
        last_pos = pos;
    }
    return true;
}



bool verify_solution_consistency(
    const std::unordered_map<int, std::vector<std::pair<int, int>>>& agent_paths,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& goals,
    const std::vector<std::vector<char>>& map) {
    for (const auto& [agent_id, path] : agent_paths) {
        std::cout << "[LNS] Verifying Agent " << agent_id << " path" << std::endl;
        if (static_cast<size_t>(agent_id) >= starts.size() || static_cast<size_t>(agent_id) >= goals.size()) {
            std::cout << "[LNS] ERROR: Agent " << agent_id << " exceeds provided start/goal data" << std::endl;
            return false;
        }
        if (path.empty()) {
            std::cout << "[LNS] ERROR: Agent " << agent_id << " has an empty path" << std::endl;
            return false;
        }
        if (path.front() != starts[agent_id]) {
            std::cout << "[LNS] ERROR: Agent " << agent_id << " does not start at the start position" << std::endl;
            return false;
        }
        if (path.back() != goals[agent_id]) {
            std::cout << "[LNS] ERROR: Agent " << agent_id << " does not end at the goal position" << std::endl;
            return false;
        }
        if (!verify_path_consistency(path, map)) {
            std::cout << "[LNS] ERROR: Agent " << agent_id << " path is not consistent" << std::endl;
            return false;
        }
    }
    return true;
}