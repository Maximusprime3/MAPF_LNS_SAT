#pragma once

#include "Current_Solution.h"
#include <vector>
#include <set>
#include <utility>

// Represents a diamond shaped conflict bucket.  Each bucket stores the set of
// grid positions that belong to the diamond as well as the indices of the
// conflicts contained within the bucket.  Buckets are used to isolate regions
// of the map for local solving.
struct DiamondBucket {
    std::set<std::pair<int,int>> positions;
    std::vector<int> indices;
    std::vector<std::vector<char>> masked_map;
    int earliest_t; // earliest timestep among conflicts in this bucket
    int latest_t; // latest timestep among conflicts in this bucket
};

// Creates a masked version of the map keeping only the cells within
// the provided shape walkable. All other cells are marked as '@'.
std::vector<std::vector<char>> mask_map_outside_shape(
    const std::vector<std::vector<char>>& map,
    const std::set<std::pair<int,int>>& shape_positions);

// Builds a shape by expanding diamond regions around conflict points.
// The expansion_radius controls how far each conflict is expanded. restricted to map bounds using the map itself
std::set<std::pair<int,int>> create_shape_from_conflicts(
    const std::vector<std::pair<int,int>>& conflict_points,
    int expansion_radius,
    const std::vector<std::vector<char>>& map);

// Find positions that are in current_shape but not in previous_shape restricted to map bounds using the map itself
std::set<std::pair<int,int>> find_new_positions(
    const std::set<std::pair<int,int>>& current_shape,
    const std::set<std::pair<int,int>>& previous_shape,
    const std::vector<std::vector<char>>& map);