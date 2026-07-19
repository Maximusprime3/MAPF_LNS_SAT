#include "Local_Zone.h" //expand_bucket_zone
#include "Current_Solution.h"
#include "Waiting_time_Solve.h"
#include "Solve_Local_Zone.h"
#include "ExperimentLogger.h"
#include "Grid.h"

#include <algorithm>

// Helper function to find agents present in a zone within a time window
std::set<int> get_agents_in_zone_within_time_window(
    const CurrentSolution& current_solution,
    const std::set<std::pair<int,int>>& zone_positions_set,
    int start_t,
    int end_t) {
    
    std::set<int> agents_in_window;
    for (const auto& rc : zone_positions_set) {
        int r = rc.first, c = rc.second;
        for (int t = start_t; t <= end_t; ++t) {
            auto here = current_solution.get_agents_at_position_time(r, c, t);
            agents_in_window.insert(here.begin(), here.end());
        }
    }
    return agents_in_window;
}



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
    int makespan_attempt_index) {
        
    LocalZoneResult local_zone_result;

    auto& logger = ExperimentLogger::instance();
    int zone_attempt_index = 0;
    auto record_zone_attempt = [&](LocalZoneAttemptMetrics zone_metrics,
                                   const WaitingSolveResult& waiting_result,
                                   int zone_index) {
        zone_metrics.waiting_attempts = static_cast<int>(waiting_result.waiting_attempts.size());
        for (const auto& attempt : waiting_result.waiting_attempts) {
            zone_metrics.total_lazy_iterations += static_cast<int>(attempt.lazy_metrics.iterations.size());
            zone_metrics.total_cnf_clauses += attempt.cnf_clauses;
            zone_metrics.total_cnf_variables += attempt.cnf_variables;
            zone_metrics.total_mdd_build_time_ms += attempt.mdd_build_time_ms;
            zone_metrics.total_cnf_build_time_ms += attempt.cnf_build_time_ms;
            zone_metrics.total_lazy_wall_time_ms += attempt.lazy_metrics.total_wall_time_ms;
            zone_metrics.total_lazy_solver_wall_time_ms += attempt.lazy_metrics.total_solver_wall_time_ms;
            zone_metrics.total_lazy_solver_reported_time_ms += attempt.lazy_metrics.total_solver_reported_time_ms;

            logger.log_waiting_attempt(experiment_id, makespan_attempt_index, zone_index, attempt);
            for (const auto& iteration_metric : attempt.lazy_metrics.iterations) {
                logger.log_lazy_iteration(experiment_id, makespan_attempt_index, zone_index, attempt.attempt_index, iteration_metric);
            }
        }
        zone_metrics.solved = waiting_result.solved();
        logger.log_local_zone_attempt(experiment_id, makespan_attempt_index, zone_metrics);
        local_zone_result.attempt_metrics.push_back(zone_metrics);
        return zone_metrics;
    };
    

    //get number of all walkable positions in the map
    const std::set<std::pair<int,int>> all_walkable_positions_set =
        mapf::walkable_positions(map);
    const std::size_t all_walkable_positions =
        all_walkable_positions_set.size();
    if (all_walkable_positions == 0) {
        std::cout << "[Solve_local_zone] ERROR: Map has no walkable positions" << std::endl;
        local_zone_result.status = SolveStatus::InvalidInput;
        local_zone_result.message = "Map has no walkable positions";
        return local_zone_result;
    }
    std::set<std::pair<int,int>> local_zone_positions = best_bucket.positions;
    std::vector<int> local_zone_conflict_indices = best_bucket.indices;
    int earliest_conflict_t = best_bucket.earliest_t;
    int latest_conflict_t = best_bucket.latest_t;
    //const int bucket_time_window_start = std::max(best_bucket.earliest_t - offset, 0);
    //const int bucket_time_window_end = std::min(best_bucket.latest_t + offset, current_max_timesteps);

    int expansion_radius = neighborhood_policy.initial_radius;
    int failed_attempt_count = 0;
    //loop until solution found or the local zone reached the size of the map and still no solution found
    while (!local_zone_result.solved() && local_zone_positions.size() <= all_walkable_positions) {
        
        if (local_zone_positions.size() > all_walkable_positions) {
            std::cout << "[Solve_local_zone] Local zone size reached all walkable positions" << std::endl;
            break;
        }
        if(local_zone_positions.size() >=0.95*all_walkable_positions) {
            std::cout << "[Solve_local_zone] Local zone size reached 95% of all walkable positions" << std::endl;
            std::cout << "[Solve_local_zone] Will try to solve with full time window and all positions" << std::endl;
            
            local_zone_positions = all_walkable_positions_set;
            local_zone_conflict_indices.clear();
            local_zone_conflict_indices.reserve(conflict_meta.size());
            for (int idx = 0; idx < static_cast<int>(conflict_meta.size()); ++idx) {
                local_zone_conflict_indices.push_back(idx);
            }
            const int full_time_window_start = 0;
            const int full_time_window_end = current_max_timesteps;
            //makespan
            std::cout << "[Solve_local_zone] Makespan: " << current_max_timesteps << std::endl;
            std::cout << "[Solve_local_zone] Time window: [" << full_time_window_start << ", " << full_time_window_end << "]" << std::endl;
            LocalZoneAttemptMetrics zone_metrics;
            zone_metrics.attempt_index = zone_attempt_index;
            zone_metrics.zone_positions = local_zone_positions.size();
            zone_metrics.zone_fraction = static_cast<double>(local_zone_positions.size()) / all_walkable_positions;
            zone_metrics.conflicts = static_cast<int>(local_zone_conflict_indices.size());
            zone_metrics.start_t = full_time_window_start;
            zone_metrics.end_t = full_time_window_end;
            zone_metrics.agents_in_window = static_cast<int>(get_agents_in_zone_within_time_window(
                current_solution, local_zone_positions, full_time_window_start, full_time_window_end).size());

            auto full_waiting_result = lazy_solve_with_waiting_time(
                current_solution,
                map,
                map,
                local_zone_positions,
                conflict_meta,
                local_zone_conflict_indices,
                conflict_map,
                full_time_window_start,
                full_time_window_end,
                expansion_radius,
                0,
                rng);
            zone_metrics = record_zone_attempt(std::move(zone_metrics), full_waiting_result, zone_attempt_index);
            zone_attempt_index++;

            if (full_waiting_result.status == SolveStatus::InvalidInput ||
                full_waiting_result.status == SolveStatus::InvalidState) {
                local_zone_result.status = full_waiting_result.status;
                local_zone_result.message = "Full-zone slack solve failed: " +
                                            full_waiting_result.message;
                break;
            }
            if (full_waiting_result.solved()) {
                std::cout << "[Solve_local_zone] Successfully solved global zone" << std::endl;
                local_zone_result.status = SolveStatus::Solved;
                local_zone_result.message = "Full-zone repair solved";
                local_zone_result.local_paths = full_waiting_result.local_paths;
                local_zone_result.local_entry_exit_time = full_waiting_result.local_entry_exit_time;
                break;
            }

            std::cout << "[Solve_local_zone] Failed to solve full map with current makespan" << std::endl;
            // The caller will respond by increasing the makespan.
            break;
        }
        //current local zone size and % or all walkable positions
        std::cout << "[Solve_local_zone] Current local zone size: " << local_zone_positions.size() << " (" 
        << (double)local_zone_positions.size() / all_walkable_positions * 100 << "% of all walkable positions)" << std::endl;
        
        //Step 1: create local problem
        auto local_masked_map = mask_map_outside_shape(map, local_zone_positions);//all positions outside the local zone are not walkable
        //set start and end time for the local zone
        int expanded_offset = expansion_radius;
        int start_t = std::max(0, earliest_conflict_t - expanded_offset);
        int end_t = std::min(current_max_timesteps, latest_conflict_t + expanded_offset);
        std::cout << "[Solve_local_zone] Local zone time window: [" << start_t << ", " << end_t << "]" << std::endl;
        
        //Step 2: solve the local problem
        auto waiting_result = lazy_solve_with_waiting_time(
            current_solution,
            map,
            local_masked_map,
            local_zone_positions,
            conflict_meta,
            local_zone_conflict_indices,
            conflict_map,
            start_t, end_t,
            expanded_offset,
            0, //initial waiting time amount
            rng);
        LocalZoneAttemptMetrics zone_metrics;
        zone_metrics.attempt_index = zone_attempt_index;
        zone_metrics.zone_positions = local_zone_positions.size();
        zone_metrics.zone_fraction = static_cast<double>(local_zone_positions.size()) / all_walkable_positions;
        zone_metrics.conflicts = static_cast<int>(local_zone_conflict_indices.size());
        zone_metrics.start_t = start_t;
        zone_metrics.end_t = end_t;
        zone_metrics.agents_in_window = static_cast<int>(get_agents_in_zone_within_time_window(
            current_solution, local_zone_positions, start_t, end_t).size());
        zone_metrics = record_zone_attempt(std::move(zone_metrics), waiting_result, zone_attempt_index);
        zone_attempt_index++;
        
        //if solution found, update the current solution
        if (waiting_result.status == SolveStatus::InvalidInput ||
            waiting_result.status == SolveStatus::InvalidState) {
            local_zone_result.status = waiting_result.status;
            local_zone_result.message = "Local slack solve failed: " + waiting_result.message;
            break;
        }
        if (waiting_result.solved()) {
            std::cout << "[Solve_local_zone] Successfully solved local zone" << std::endl;
            //integration of local zone result into current solution happens in waiting time solve
            local_zone_result.status = SolveStatus::Solved;
            local_zone_result.message = "Local-zone repair solved";
            local_zone_result.local_paths = waiting_result.local_paths;
            local_zone_result.local_entry_exit_time = waiting_result.local_entry_exit_time;
            break;
        }

        //if no solution found
        //Step 3: Expand the local zone 
        // Expansion attempts: increase bucket offset and try again
        std::cout << "[Solve_local_zone] Zone with radius " << expansion_radius
                  << " failed" << std::endl;

        ++failed_attempt_count;
        expansion_radius = next_neighborhood_radius(
            neighborhood_policy, expansion_radius, failed_attempt_count);

        expanded_offset = expansion_radius;
        std::cout << "[Solve_local_zone] Expanding zone to radius "
                  << expanded_offset << std::endl;
        
        //update the time window for the bucket with the new expansion factor
        const int bucket_time_window_start = std::max(earliest_conflict_t - expanded_offset, 0);
        const int bucket_time_window_end = std::min(latest_conflict_t + expanded_offset, current_max_timesteps);

        
        // Recreate the bucket conflicts from the original bucket indices
        // Use helper to expand zone and gather expanded conflict indices
        //todo:no more conflict points? all conflict meta
        auto [expanded_zone_positions_set, expanded_conflict_indices] = expand_bucket_zone(
            conflict_meta,
            conflict_map,
            map,
            local_zone_conflict_indices,
            local_zone_positions,
            expanded_offset,
            bucket_time_window_start,
            bucket_time_window_end);
        
        local_zone_positions = expanded_zone_positions_set;
        local_zone_conflict_indices = expanded_conflict_indices; //todo: no more conflict indices? all conflict meta
        //find earliest and latest conflict times in the expanded zone
        for (int conflict_idx : expanded_conflict_indices) {
            if (conflict_idx >= 0 && conflict_idx < (int)conflict_meta.size()) {
                int t = conflict_meta[conflict_idx].timestep;
                if (t < earliest_conflict_t) earliest_conflict_t = t;
                if (t > latest_conflict_t) latest_conflict_t = t;
            }else{
                std::cout << "[Solve_local_zone] ERROR: Invalid conflict index " << conflict_idx << std::endl;
            }
        }

        earliest_conflict_t = std::max(earliest_conflict_t, bucket_time_window_start);
        latest_conflict_t = std::min(latest_conflict_t, bucket_time_window_end);

        start_t = std::max(0, earliest_conflict_t - expanded_offset);
        end_t = std::min(current_max_timesteps, latest_conflict_t + expanded_offset);

        std::cout << "[Solve_local_zone] Final expanded zone contains " << expanded_zone_positions_set.size() 
                  << " positions with " << expanded_conflict_indices.size() << " conflicts " << std::endl; 
        std::cout << "[Solve_local_zone] New time window: [" << start_t << ", " << end_t << "]" << std::endl;
        

    }
    if (local_zone_result.status == SolveStatus::Exhausted &&
        local_zone_result.message.empty()) {
        local_zone_result.message = "Zone expansion exhausted at the current makespan";
    }
    std::cout << "[Solve_local_zone] Final local zone size: " << local_zone_positions.size() << " (" << (double)local_zone_positions.size() / all_walkable_positions * 100 << "% of all walkable positions)" << std::endl;
    return local_zone_result;
}
