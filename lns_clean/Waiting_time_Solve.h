#pragma once

#include "Create_Local_Problem.h"
#include "Current_Solution.h"
#include "Lazy_SAT_Solve.h"
#include "SolverConfiguration.h"

#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

// Result of the slack-search layer. It deliberately owns only the payload
// needed by zone expansion; SAT-specific collision details remain in the
// individual LazySolveResult recorded by each attempt's metrics.
struct WaitingSolveResult {
    SolveStatus status = SolveStatus::Exhausted;
    std::string message;
    std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths;
    std::unordered_map<int, std::pair<int,int>> local_entry_exit_time;
    std::vector<WaitingAttemptMetrics> waiting_attempts;

    bool solved() const {
        return status == SolveStatus::Solved;
    }
};

bool apply_waiting_time_delta(
    LocalZoneState& state,
    int segment_id,
    int original_id,
    int waiting_time_delta,
    const std::vector<std::vector<char>>& masked_map,
    const std::vector<std::vector<char>>& map,
    CurrentSolution& current_solution,
    std::mt19937& rng);

WaitingSolveResult lazy_solve_with_waiting_time(
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
    int initial_waiting_time_amount,
    std::mt19937& rng,
    const SolverConfig& config,
    const SolverDeadline& deadline);


WaitingSolveResult lazy_solve_with_waiting_time(
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
    int initial_waiting_time_amount,
    std::mt19937& rng);
