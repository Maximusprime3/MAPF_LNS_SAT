#include "LNSGeometry.h"

#include <cmath>

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
