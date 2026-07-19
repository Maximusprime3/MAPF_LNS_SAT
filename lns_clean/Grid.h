#pragma once

#include <cstddef>
#include <set>
#include <utility>
#include <vector>

namespace mapf {

// Moving AI map rows are preserved verbatim by the loader. The solver treats
// only normal ground and goal terrain as traversable; every other byte is
// blocked, including @, O, T, S, W, and unknown terrain symbols.
inline bool is_walkable_cell(char cell) noexcept {
    return cell == '.' || cell == 'G';
}

inline bool is_in_bounds(
    const std::vector<std::vector<char>>& grid,
    int row,
    int column) noexcept {
    return row >= 0 && row < static_cast<int>(grid.size()) &&
           column >= 0 &&
           column < static_cast<int>(grid[static_cast<std::size_t>(row)].size());
}

inline bool is_in_bounds(
    const std::vector<std::vector<char>>& grid,
    const std::pair<int, int>& position) noexcept {
    return is_in_bounds(grid, position.first, position.second);
}

inline bool is_walkable_position(
    const std::vector<std::vector<char>>& grid,
    int row,
    int column) noexcept {
    return is_in_bounds(grid, row, column) &&
           is_walkable_cell(
               grid[static_cast<std::size_t>(row)]
                   [static_cast<std::size_t>(column)]);
}

inline bool is_walkable_position(
    const std::vector<std::vector<char>>& grid,
    const std::pair<int, int>& position) noexcept {
    return is_walkable_position(grid, position.first, position.second);
}

inline std::set<std::pair<int, int>> walkable_positions(
    const std::vector<std::vector<char>>& grid) {
    std::set<std::pair<int, int>> positions;
    for (std::size_t row = 0; row < grid.size(); ++row) {
        for (std::size_t column = 0; column < grid[row].size(); ++column) {
            if (is_walkable_cell(grid[row][column])) {
                positions.emplace(
                    static_cast<int>(row), static_cast<int>(column));
            }
        }
    }
    return positions;
}

}  // namespace mapf
