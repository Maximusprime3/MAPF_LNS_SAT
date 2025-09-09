#pragma once

#include <vector>
#include <set>
#include <utility>

// Creates a masked version of the map keeping only the cells within
// the provided shape walkable. All other cells are marked as '@'.
std::vector<std::vector<char>> mask_map_outside_shape(
    const std::vector<std::vector<char>>& map,
    const std::set<std::pair<int,int>>& shape_positions);

// Builds a shape by expanding diamond regions around conflict points.
// The expansion_radius controls how far each conflict is expanded.
std::set<std::pair<int,int>> create_shape_from_conflicts(
    const std::vector<std::pair<int,int>>& conflict_points,
    int expansion_radius);
