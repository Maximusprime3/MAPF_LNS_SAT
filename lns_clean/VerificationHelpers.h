#ifndef LNS_VERIFICATION_HELPERS_H
#define LNS_VERIFICATION_HELPERS_H

#include <unordered_map>
#include <utility>
#include <vector>

// Forward declaration for map representation is not needed as we include vector header

bool verify_path_consistency(
    const std::vector<std::pair<int, int>>& path,
    const std::vector<std::vector<char>>& map);

// Verifies that each agent path starts and ends at the correct positions and
// that all intermediate steps stay within map bounds and on walkable cells.
bool verify_solution_consistency(
    const std::unordered_map<int, std::vector<std::pair<int, int>>>& agent_paths,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& goals,
    const std::vector<std::vector<char>>& map);

#endif // LNS_VERIFICATION_HELPERS_H