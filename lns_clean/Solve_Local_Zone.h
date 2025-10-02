#pragma once

#include "Local_Zone.h"
#include "Current_Solution.h"

#include <random>
#include <unordered_map>
#include <vector>
#include <utility>


struct LocalZoneResult {
    bool solution_found = false;
    std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths;
    std::unordered_map<int, std::pair<int,int>> local_entry_exit_time;
};


LocalZoneResult solve_local_zone(
    const std::vector<std::vector<char>>& map,
    const DiamondBucket& best_bucket,
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    CurrentSolution& current_solution,
    int offset,
    int current_max_timesteps,
    std::mt19937& rng);