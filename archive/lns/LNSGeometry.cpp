#include "LNSGeometry.h"

#include <cmath>
#include <algorithm>  // for std::set_difference
#include <iterator>   // for std::inserter

std::vector<std::vector<char>> mask_map_outside_shape(
    const std::vector<std::vector<char>>& map,
    const std::set<std::pair<int,int>>& shape_positions) {
    if (map.empty() || map[0].empty()) return {};
    int rows = static_cast<int>(map.size());
    int cols = static_cast<int>(map[0].size());

    // Start with a map filled with obstacles and carve out the shape.
    std::vector<std::vector<char>> masked(rows, std::vector<char>(cols, '@'));

    for (const auto& pos : shape_positions) {
        int r = pos.first;
        int c = pos.second;
        if (r >= 0 && r < rows && c >= 0 && c < cols) {
            masked[r][c] = map[r][c];
        }
    }

    return masked;
}

std::set<std::pair<int,int>> create_shape_from_conflicts(
    const std::vector<std::pair<int,int>>& conflict_points,
    int expansion_radius) {
    std::set<std::pair<int,int>> shape_positions;

    for (const auto& conflict_point : conflict_points) {
        int center_row = conflict_point.first;
        int center_col = conflict_point.second;

        for (int r = center_row - expansion_radius; r <= center_row + expansion_radius; ++r) {
            for (int c = center_col - expansion_radius; c <= center_col + expansion_radius; ++c) {
                if (std::abs(r - center_row) + std::abs(c - center_col) <= expansion_radius) {
                    shape_positions.insert({r, c});
                }
            }
        }
    }

    return shape_positions;
}

// Overload: restricted to map bounds (rows x cols)
std::set<std::pair<int,int>> create_shape_from_conflicts(
    const std::vector<std::pair<int,int>>& conflict_points,
    int expansion_radius,
    int rows,
    int cols) {
    std::set<std::pair<int,int>> shape_positions;

    for (const auto& conflict_point : conflict_points) {
        int center_row = conflict_point.first;
        int center_col = conflict_point.second;

        for (int r = center_row - expansion_radius; r <= center_row + expansion_radius; ++r) {
            for (int c = center_col - expansion_radius; c <= center_col + expansion_radius; ++c) {
                if (r < 0 || r >= rows || c < 0 || c >= cols) continue;
                if (std::abs(r - center_row) + std::abs(c - center_col) <= expansion_radius) {
                    shape_positions.insert({r, c});
                }
            }
        }
    }

    return shape_positions;
}

// Overload: restrict using the map itself
std::set<std::pair<int,int>> create_shape_from_conflicts(
    const std::vector<std::pair<int,int>>& conflict_points,
    int expansion_radius,
    const std::vector<std::vector<char>>& map) {
    int rows = (int)map.size();
    int cols = rows > 0 ? (int)map[0].size() : 0;
    std::set<std::pair<int,int>> shape_positions;

    for (const auto& conflict_point : conflict_points) {
        int center_row = conflict_point.first;
        int center_col = conflict_point.second;

        for (int r = center_row - expansion_radius; r <= center_row + expansion_radius; ++r) {
            for (int c = center_col - expansion_radius; c <= center_col + expansion_radius; ++c) {
                if (r < 0 || r >= rows || c < 0 || c >= cols) continue;
                if (std::abs(r - center_row) + std::abs(c - center_col) <= expansion_radius) {
                    // Only include walkable ('.') or goal ('G') cells
                    char cell = map[r][c];
                    if (cell == '.' || cell == 'G') {
                        shape_positions.insert({r, c});
                    }
                }
            }
        }
    }

    return shape_positions;
}

// Helper: find positions that are in current_shape but not in previous_shape
// This is more efficient than manually iterating and checking membership
std::set<std::pair<int,int>> find_new_positions(
    const std::set<std::pair<int,int>>& current_shape,
    const std::set<std::pair<int,int>>& previous_shape) {
    std::set<std::pair<int,int>> new_positions;
    
    // Use set_difference for efficient computation
    std::set_difference(
        current_shape.begin(), current_shape.end(),
        previous_shape.begin(), previous_shape.end(),
        std::inserter(new_positions, new_positions.begin())
    );
    
    return new_positions;
}

// Overload: restricted to map bounds (rows x cols)
std::set<std::pair<int,int>> find_new_positions(
    const std::set<std::pair<int,int>>& current_shape,
    const std::set<std::pair<int,int>>& previous_shape,
    int rows,
    int cols) {
    // First compute the difference
    auto diff = find_new_positions(current_shape, previous_shape);
    // Then filter by bounds
    std::set<std::pair<int,int>> bounded;
    for (const auto& pos : diff) {
        int r = pos.first, c = pos.second;
        if (r >= 0 && r < rows && c >= 0 && c < cols) {
            bounded.insert(pos);
        }
    }
    return bounded;
}

// Overload: restrict using the map itself
std::set<std::pair<int,int>> find_new_positions(
    const std::set<std::pair<int,int>>& current_shape,
    const std::set<std::pair<int,int>>& previous_shape,
    const std::vector<std::vector<char>>& map) {
    int rows = (int)map.size();
    int cols = rows > 0 ? (int)map[0].size() : 0;
    // First compute the set difference
    auto diff = find_new_positions(current_shape, previous_shape);
    // Then filter by bounds and walkability
    std::set<std::pair<int,int>> bounded;
    for (const auto& pos : diff) {
        int r = pos.first, c = pos.second;
        if (r >= 0 && r < rows && c >= 0 && c < cols) {
            char cell = map[r][c];
            if (cell == '.' || cell == 'G') {
                bounded.insert(pos);
            }
        }
    }
    return bounded;
}
