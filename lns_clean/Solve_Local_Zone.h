#pragma once

#include "Local_Zone.h"
#include "Current_Solution.h"
#include "Metrics.h"
#include "NeighborhoodVariant.h"
#include "SolverConfiguration.h"
#include "SolveStatus.h"

#include <random>
#include <unordered_map>
#include <vector>
#include <utility>


struct LocalZoneResult {
    SolveStatus status = SolveStatus::Exhausted;
    std::string message;
    std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths;
    std::unordered_map<int, std::pair<int,int>> local_entry_exit_time;
    std::vector<LocalZoneAttemptMetrics> attempt_metrics;

    bool solved() const {
        return status == SolveStatus::Solved;
    }
};


LocalZoneResult solve_local_zone(
    const std::vector<std::vector<char>>& map,
    const DiamondBucket& best_bucket,
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    CurrentSolution& current_solution,
    const NeighborhoodPolicy& neighborhood_policy,
    int current_max_timesteps,
    std::mt19937& rng,
    const std::string& experiment_id,
    int makespan_attempt_index,
    const SolverConfig& config,
    const SolverDeadline& deadline);


LocalZoneResult solve_local_zone(
    const std::vector<std::vector<char>>& map,
    const DiamondBucket& best_bucket,
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    CurrentSolution& current_solution,
    const NeighborhoodPolicy& neighborhood_policy,
    int current_max_timesteps,
    std::mt19937& rng,
    const std::string& experiment_id,
    int makespan_attempt_index);
