#pragma once

#include "Create_Local_Problem.h"
#include "Current_Solution.h"
#include "lazy_SAT_Solve.h"

#include <set>
#include <vector>

void apply_waiting_time_delta(LocalZoneState& state,
    int segment_id,
    int delta,
    const std::vector<std::vector<char>>& masked_map);

LazySolveResult lazy_solve_with_waiting_time(
    CurrentSolution& current_solution,
    const std::vector<std::vector<char>>& map,
    const std::vector<std::vector<char>>& masked_map,
    const std::set<std::pair<int,int>>& local_zone_positions,
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<int>& local_zone_conflict_indices,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    int start_t,
    int end_t,
    int offset,
    int initial_waiting_time_amount = 0);