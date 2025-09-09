#include "LNSHelpers.h"
#include "../SATSolverManager.h"
#include <vector>
#include <string>
#include <iostream>
#include <optional>
#include <random>
#include <set>
#include <algorithm>
#include <climits>
#include <cmath>
#include <unordered_map>

// ============================================================================
// REFACTORED LNS SOLVER USING HELPER CLASSES
// ============================================================================

// Entry point for refactored LNS solver
int run_refactored_lns(const std::string& map_path,
                      const std::string& scenario_path,
                      int num_agents,
                      int scenario_index /*0-based*/,
                      bool use_minisat /*else ProbSAT*/,
                      int seed) {
    
    std::cout << "[LNS-Refactored] Starting refactored LNS solver..." << std::endl;
    
    // Step 1: Load problem
    auto problem_opt = load_problem(map_path, scenario_path, num_agents, scenario_index);
    if (!problem_opt.has_value()) {
        return 1;
    }
    const auto& problem = problem_opt.value();
    
    std::cout << "[LNS-Refactored] Loaded map " << problem.grid.size() << "x"
              << (problem.grid.empty() ? 0 : (int)problem.grid[0].size())
              << ", agents: " << problem.starts.size() << std::endl;
    
    // Step 2: Compute base makespan
    auto [distance_matrices, base_makespan] = SATSolverManager::compute_max_timesteps(
        problem.grid, problem.starts, problem.goals);
    if (base_makespan <= 0) base_makespan = 1;
    
    // Initialize helper classes
    int rows = (int)problem.grid.size();
    int cols = (int)problem.grid[0].size();
    int num_agents_int = (int)problem.starts.size();
    
    CollisionTracker global_collision_tracker;
    ConflictBucketManager bucket_manager(rows, cols, 2); // offset = 2
    MDDManager mdd_manager(problem.grid);
    ZoneExpansionManager expansion_manager(rows, cols);
    PathManager path_manager;
    WaitingTimeStrategyManager waiting_time_manager;
    
    // Outer loop: increase max timesteps if no solution is found
    const int max_timestep_increase = 10;
    std::mt19937 rng(static_cast<unsigned int>(seed));
    
    for (int inc = 0; inc <= max_timestep_increase; ++inc) {
        int current_max_timesteps = base_makespan + inc;
        std::cout << "\n[LNS-Refactored] === Attempt with max_timesteps=" << current_max_timesteps << " ===" << std::endl;
        
        // Clear collision tracker for this makespan attempt
        global_collision_tracker.clear();
        
        // Build MDDs with waiting time structure
        auto mdds = create_mdds_with_waiting_time(
            problem.grid, problem.starts, problem.goals, current_max_timesteps, distance_matrices);
        std::cout << "[LNS-Refactored] Built MDDs with waiting time structure for " << mdds.size()
                  << " agents at makespan " << current_max_timesteps << std::endl;
        
        // Step 1: Sample paths and create current solution
        std::cout << "[LNS-Refactored] Step 1: Sampling paths and creating current solution..." << std::endl;
        CurrentSolution current_solution(rows, cols, current_max_timesteps, num_agents_int);
        
        // Sample paths using PathManager
        auto sampled_paths = path_manager.sample_paths_from_mdds(mdds, rng);
        if (!path_manager.validate_paths(sampled_paths)) {
            std::cerr << "[ERROR] Invalid paths sampled" << std::endl;
            continue;
        }
        
        // Update current solution with sampled paths
        for (const auto& [agent_id, path] : sampled_paths) {
            current_solution.agent_paths[agent_id] = path;
        }
        
        // Calculate waiting times and create path map
        current_solution.calculate_waiting_times(problem.goals, current_max_timesteps);
        current_solution.create_path_map();
        
        std::cout << "[LNS-Refactored] Created current solution with " << current_solution.agent_paths.size() 
                  << " agent paths" << std::endl;
        
        // Main LNS iteration loop
        int iteration = 0;
        const int max_iterations = 100;
        
        while (iteration < max_iterations) {
            iteration++;
            std::cout << "\n[LNS-Refactored] === Iteration " << iteration << " ===" << std::endl;
            
            // Step 2: Conflict analysis
            std::cout << "[LNS-Refactored] Step 2: Analyzing conflicts..." << std::endl;
            auto [vertex_collisions, edge_collisions] = SATSolverManager::find_all_collisions(current_solution.agent_paths);
            
            // Add collisions to global tracker
            global_collision_tracker.add_vertex_collisions(vertex_collisions);
            global_collision_tracker.add_edge_collisions(edge_collisions);
            
            std::cout << "[LNS-Refactored] Current solution: vertex collisions=" << vertex_collisions.size()
                      << ", edge collisions=" << edge_collisions.size() << std::endl;
            
            // If no conflicts, we're done
            if (vertex_collisions.empty() && edge_collisions.empty()) {
                std::cout << "[LNS-Refactored] Collision-free solution found at makespan " << current_max_timesteps << std::endl;
                std::cout << "\n[LNS-Refactored] Final agent paths:" << std::endl;
                SATSolverManager::print_agent_paths(current_solution.agent_paths);
                return 0;
            }
            
            // Step 3: Create conflict buckets
            std::cout << "[LNS-Refactored] Step 3: Creating conflict buckets..." << std::endl;
            
            // Prepare conflict data
            std::vector<std::pair<int,int>> conflict_points;
            std::vector<ConflictMeta> conflict_meta;
            conflict_points.reserve(vertex_collisions.size() + edge_collisions.size() * 2);
            conflict_meta.reserve(vertex_collisions.size() + edge_collisions.size() * 2);
            
            // Process vertex collisions
            for (const auto& v : vertex_collisions) {
                int a1 = std::get<0>(v);
                int a2 = std::get<1>(v);
                auto pos = std::get<2>(v);
                int t = std::get<3>(v);
                conflict_points.push_back(pos);
                conflict_meta.push_back(ConflictMeta{a1, a2, t, false, pos, {-1,-1}});
            }
            
            // Process edge collisions
            for (const auto& e : edge_collisions) {
                int a1 = std::get<0>(e);
                int a2 = std::get<1>(e);
                auto pos1 = std::get<2>(e);
                auto pos2 = std::get<3>(e);
                int t = std::get<4>(e);
                conflict_points.push_back(pos1);
                conflict_meta.push_back(ConflictMeta{a1, a2, t, true, pos1, pos2});
                conflict_points.push_back(pos2);
                conflict_meta.push_back(ConflictMeta{a1, a2, t, true, pos1, pos2});
            }
            
            // Create conflict buckets using ConflictBucketManager
            std::set<int> solved_conflict_indices; // Empty for now
            auto diamond_buckets = bucket_manager.create_diamond_buckets(
                conflict_points, conflict_meta, solved_conflict_indices);
            
            bucket_manager.print_bucket_statistics(diamond_buckets);
            
            if (diamond_buckets.empty()) {
                std::cout << "[LNS-Refactored] No conflict buckets created" << std::endl;
                break;
            }
            
            // Step 4: Process conflict buckets
            std::cout << "[LNS-Refactored] Step 4: Processing conflict buckets..." << std::endl;
            
            for (size_t bucket_idx = 0; bucket_idx < diamond_buckets.size(); ++bucket_idx) {
                const auto& bucket = diamond_buckets[bucket_idx];
                std::cout << "\n[LNS-Refactored] Processing bucket " << bucket_idx 
                          << " with " << bucket.positions.size() << " positions and " 
                          << bucket.indices.size() << " conflicts" << std::endl;
                
                // Validate conflict zone
                if (!validate_conflict_zone(bucket.positions, rows, cols)) {
                    std::cerr << "[ERROR] Invalid conflict zone" << std::endl;
                    continue;
                }
                
                // Create masked map for this zone
                auto masked_map = create_masked_map_for_zone(problem.grid, bucket.positions);
                
                // Find agents in this conflict zone
                auto agents_in_zone = current_solution.get_agents_in_zone(bucket.positions);
                std::cout << "[LNS-Refactored] Found " << agents_in_zone.size() 
                          << " agents in conflict zone" << std::endl;
                
                if (agents_in_zone.empty()) {
                    std::cout << "[LNS-Refactored] No agents in conflict zone, skipping bucket" << std::endl;
                    continue;
                }
                
                // Extract local paths using PathManager
                int start_t = 0, end_t = current_max_timesteps; // Simplified time window
                auto local_path_data = path_manager.extract_local_paths(
                    current_solution, bucket.positions, agents_in_zone, start_t, end_t);
                
                if (local_path_data.local_zone_paths.empty()) {
                    std::cout << "[LNS-Refactored] No local paths extracted, skipping bucket" << std::endl;
                    continue;
                }
                
                // Create MDDs for agents in zone using MDDManager
                auto local_mdds = mdd_manager.create_zone_mdds(
                    local_path_data.local_zone_paths, 
                    local_path_data.local_entry_exit_time,
                    masked_map, start_t, end_t);
                
                if (local_mdds.empty()) {
                    std::cout << "[LNS-Refactored] No MDDs created for zone, skipping bucket" << std::endl;
                    continue;
                }
                
                // Try waiting time strategy first
                std::cout << "[LNS-Refactored] Trying waiting time strategy..." << std::endl;
                
                CollisionTracker initial_collisions;
                auto waiting_time_result = waiting_time_manager.execute_strategy(
                    current_solution, masked_map, bucket.positions, local_mdds,
                    local_path_data.local_zone_paths, local_path_data.local_entry_exit_time,
                    initial_collisions, conflict_meta, bucket.indices, start_t, end_t);
                
                if (waiting_time_result.solution_found) {
                    std::cout << "[LNS-Refactored] Waiting time strategy succeeded!" << std::endl;
                    
                    // Update global solution
                    path_manager.update_global_solution(
                        current_solution, waiting_time_result.local_paths, 
                        local_path_data.local_entry_exit_time);
                    
                    // Merge discovered collisions
                    global_collision_tracker.merge_collisions(waiting_time_result.discovered_collisions);
                    
                    // Mark conflicts as solved
                    for (int idx : bucket.indices) {
                        solved_conflict_indices.insert(idx);
                    }
                    
                    std::cout << "[LNS-Refactored] Successfully resolved bucket " << bucket_idx << std::endl;
                    continue; // Move to next bucket
                }
                
                // If waiting time strategy failed, try zone expansion
                std::cout << "[LNS-Refactored] Waiting time strategy failed, trying zone expansion..." << std::endl;
                
                bool expansion_success = false;
                int expansion_factor = 1;
                const int max_expansion_factor = 5; // Limit expansion attempts
                
                while (!expansion_success && expansion_factor <= max_expansion_factor) {
                    std::cout << "[LNS-Refactored] Trying expansion factor " << expansion_factor << std::endl;
                    
                    // Create conflict map for efficient queries
                    auto conflict_map = bucket_manager.create_conflict_map(conflict_points);
                    
                    // Expand zone using ZoneExpansionManager
                    auto expansion_result = expansion_manager.expand_zone(
                        conflict_points, bucket.indices, expansion_factor, 2, // base_offset = 2
                        conflict_points, conflict_map, solved_conflict_indices);
                    
                    if (!expansion_result.success) {
                        std::cout << "[LNS-Refactored] Zone expansion failed" << std::endl;
                        break;
                    }
                    
                    // Check if expanded zone covers entire map
                    if (expansion_manager.is_global_zone(expansion_result.expanded_positions)) {
                        std::cout << "[LNS-Refactored] Expanded zone covers entire map, need to increase makespan" << std::endl;
                        goto increase_makespan;
                    }
                    
                    std::cout << "[LNS-Refactored] Expanded zone coverage: " 
                              << (expansion_manager.get_zone_coverage(expansion_result.expanded_positions) * 100.0) 
                              << "%" << std::endl;
                    
                    // Create new masked map for expanded zone
                    auto expanded_masked_map = create_masked_map_for_zone(problem.grid, expansion_result.expanded_positions);
                    
                    // Find agents in expanded zone
                    auto expanded_agents_in_zone = current_solution.get_agents_in_zone(expansion_result.expanded_positions);
                    
                    // Extract local paths for expanded zone
                    auto expanded_local_path_data = path_manager.extract_local_paths(
                        current_solution, expansion_result.expanded_positions, expanded_agents_in_zone, start_t, end_t);
                    
                    // Create MDDs for expanded zone
                    auto expanded_local_mdds = mdd_manager.create_zone_mdds(
                        expanded_local_path_data.local_zone_paths,
                        expanded_local_path_data.local_entry_exit_time,
                        expanded_masked_map, start_t, end_t);
                    
                    // Try waiting time strategy with expanded zone
                    auto expanded_waiting_time_result = waiting_time_manager.execute_strategy(
                        current_solution, expanded_masked_map, expansion_result.expanded_positions, expanded_local_mdds,
                        expanded_local_path_data.local_zone_paths, expanded_local_path_data.local_entry_exit_time,
                        initial_collisions, conflict_meta, expansion_result.expanded_conflict_indices, start_t, end_t);
                    
                    if (expanded_waiting_time_result.solution_found) {
                        std::cout << "[LNS-Refactored] Expanded zone waiting time strategy succeeded!" << std::endl;
                        
                        // Update global solution
                        path_manager.update_global_solution(
                            current_solution, expanded_waiting_time_result.local_paths,
                            expanded_local_path_data.local_entry_exit_time);
                        
                        // Merge discovered collisions
                        global_collision_tracker.merge_collisions(expanded_waiting_time_result.discovered_collisions);
                        
                        // Mark expanded conflicts as solved
                        for (int idx : expansion_result.expanded_conflict_indices) {
                            solved_conflict_indices.insert(idx);
                        }
                        
                        expansion_success = true;
                        std::cout << "[LNS-Refactored] Successfully resolved expanded bucket " << bucket_idx << std::endl;
                    } else {
                        std::cout << "[LNS-Refactored] Expanded zone waiting time strategy failed" << std::endl;
                        expansion_factor++;
                    }
                }
                
                if (!expansion_success) {
                    std::cout << "[LNS-Refactored] All expansion strategies failed for bucket " << bucket_idx << std::endl;
                }
            }
            
            // Check if we've solved all conflicts
            auto [remaining_vertex_collisions, remaining_edge_collisions] = 
                SATSolverManager::find_all_collisions(current_solution.agent_paths);
            
            if (remaining_vertex_collisions.empty() && remaining_edge_collisions.empty()) {
                std::cout << "[LNS-Refactored] All conflicts resolved in iteration " << iteration << std::endl;
                break;
            }
            
            std::cout << "[LNS-Refactored] Remaining conflicts: vertex=" << remaining_vertex_collisions.size()
                      << ", edge=" << remaining_edge_collisions.size() << std::endl;
        }
        
        // If we reach here, we couldn't solve all conflicts at this makespan
        std::cout << "[LNS-Refactored] Could not resolve all conflicts at makespan " << current_max_timesteps << std::endl;
        
        increase_makespan:
        std::cout << "[LNS-Refactored] Moving to next makespan attempt..." << std::endl;
    }
    
    std::cout << "[LNS-Refactored] Failed to find solution within makespan limit" << std::endl;
    return 1;
}

// ============================================================================
// MAIN FUNCTION - Entry point for the refactored solver
// ============================================================================

int main(int argc, char* argv[]) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <map_path> <scenario_path> <num_agents> <scenario_index> [use_minisat] [seed]" << std::endl;
        return 1;
    }
    
    std::string map_path = argv[1];
    std::string scenario_path = argv[2];
    int num_agents = std::stoi(argv[3]);
    int scenario_index = std::stoi(argv[4]);
    bool use_minisat = (argc > 5) ? (std::stoi(argv[5]) != 0) : true;
    int seed = (argc > 6) ? std::stoi(argv[6]) : 42;
    
    std::cout << "[LNS-Refactored] Starting with parameters:" << std::endl;
    std::cout << "  Map: " << map_path << std::endl;
    std::cout << "  Scenario: " << scenario_path << std::endl;
    std::cout << "  Agents: " << num_agents << std::endl;
    std::cout << "  Scenario index: " << scenario_index << std::endl;
    std::cout << "  Use MiniSat: " << (use_minisat ? "true" : "false") << std::endl;
    std::cout << "  Seed: " << seed << std::endl;
    
    return run_refactored_lns(map_path, scenario_path, num_agents, scenario_index, use_minisat, seed);
}
