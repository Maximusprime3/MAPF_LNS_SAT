#include "Load_LNSProblem.h" 
#include "Current_Solution.h" //collect_conflicts, create_conflict_map_2D
#include "Local_Zone.h" //build_diamond_buckets
#include "../SATSolverManager.h" //find_all_collisions, print_agent_paths

#include <iostream>
#include <vector>
#include <random>
#include <set>
#include <algorithm>
#include <climits>
#include <cmath>
#include <unordered_map>
#include <optional>
#include <string>
#include <iostream>

//main function for LNS
//takes map path, scenario path, number of agents, scenario index, use minisat, seed
//returns paths of agents

std::unordered_map<int, std::vector<std::vector<int,int>>> LNS(
    const std::string& map_path, 
    const std::string& scenario_path, 
    int num_agents, 
    int scenario_index, 
    bool use_minisat, //room to plug in different solvers
    int seed) {
    
    //Step 1: Load problem and print basic info
    auto problem_loaded = load_problem(map_path, scenario_path, num_agents, scenario_index);
    if (!problem_loaded.has_value()) {
        std::cerr << "[LNS] Failed to load problem" << std::endl;
        return std::unordered_map<int, std::vector<std::vector<int,int>>>();
    }
    const auto& problem = problem_loaded.value();

    std::cout << "[LNS] Loaded map " << problem.grid.size() << "x"
              << (problem.grid.empty() ? 0 : (int)problem.grid[0].size())
              << ", agents: " << problem.starts.size() << std::endl;

    //print full map
    std::cout << "[LNS] Map:" << std::endl;
    for (const auto& row : problem.grid) {
        for (char c : row) std::cout << c;
        std::cout << '\n';
    }

    //print number of agents and their starts and goals
    std::cout << "[LNS] Number of agents: " << problem.starts.size() << std::endl;
    std::cout << "[LNS] Agents (start -> goal):" << std::endl;
    for (size_t i = 0; i < problem.starts.size(); ++i) {
        const auto& s = problem.starts[i];
        const auto& g = problem.goals[i];
        std::cout << "  Agent " << i << ": (" << s.first << "," << s.second << ") -> ("
                  << g.first << "," << g.second << ")" << std::endl;
    }

    //Step 2: Compute base makespan and distance matrices for MDDs of all agents
    auto [distance_matrices, base_makespan] = SATSolverManager::compute_max_timesteps(
        problem.grid, problem.starts, problem.goals);
    if (base_makespan <= 0) base_makespan = 1;

    std::cout << "[LNS] Base makespan: " << base_makespan << std::endl;

    //Step 3: Outer loop: increase max timesteps if no solution is found
    const int max_timestep_increase = 10; // crude limit for now
    std::mt19937 rng(static_cast<unsigned int>(seed));
    
    for (int inc = 0; inc <= max_timestep_increase; ++inc) {
        int current_max_timesteps = base_makespan + inc;
        std::cout << "\n[LNS] === Attempt with max_timesteps=" << current_max_timesteps << " ===" << std::endl;

        //Build MDDs with fastest way to the goal and sample for initial solution
        //this is just for the initial solution, can be improved in the future
        auto mdds = create_mdds_with_waiting_time(
            problem.grid, problem.starts, problem.goals, current_max_timesteps, distance_matrices);
        std::cout << "[LNS] Built MDDs with waiting time structure for " << mdds.size()
                  << " agents at makespan " << current_max_timesteps << std::endl;
        
        //Step 4: create current solution by sampling paths from MDDs
        CurrentSolution current_solution(problem.grid.size(), problem.grid[0].size(), current_max_timesteps);
        for (size_t agent_id = 0; agent_id < mdds.size(); ++agent_id) {
            const auto& mdd = mdds[agent_id];
            auto path_positions = mdd->sample_random_path(rng);
            std::vector<std::pair<int,int>> as_pairs(path_positions.begin(), path_positions.end());
            current_solution.agent_paths[static_cast<int>(agent_id)] = std::move(as_pairs);
        }
        //calculate waiting times for each agent
        current_solution.calculate_waiting_times(problem.goals, current_max_timesteps);
        //create path map for current solution
        current_solution.create_path_map();
        std::cout << "[LNS] Created current solution with " << current_solution.agent_paths.size() 
                  << " agent paths on a map of size " << problem.grid.size() << "x" << problem.grid[0].size() << std::endl;

        bool conflicts_remain = true;
        while(conflicts_remain){
            //Step 5: analyze conflicts
            std::cout << "[LNS] Analyzing conflicts..." << std::endl;
            auto [vertex_collisions, edge_collisions] = SATSolverManager::find_all_collisions(current_solution.agent_paths);
            std::cout << "[LNS] Current solution: vertex collisions=" << vertex_collisions.size()
                    << ", edge collisions=" << edge_collisions.size() << std::endl;

            //If no conflicts, we're done
            if (vertex_collisions.empty() && edge_collisions.empty()) {
                std::cout << "[LNS] Collision-free solution found at makespan " << current_max_timesteps << std::endl;
                std::cout << "[LNS] Final agent paths:" << std::endl;
                SATSolverManager::print_agent_paths(current_solution.agent_paths);
                conflicts_remain = false;
                continue;
            }
            //otherwise, update metadata from detected collisions
            auto conflict_meta = collect_conflicts_meta(vertex_collisions, edge_collisions);
            auto conflict_map = create_conflict_map_2D(conflict_meta, problem.grid);
            

            //Step 6: create conflict buckets for the earliest conflict(s)
            const int offset = 1; // half the standard conflict zone size
            std::cout << "[LNS] Creating conflict buckets..." << std::endl;
            auto diamond_buckets = build_diamond_buckets_for_earliest_conflicts(
                conflict_meta, conflict_map, problem.grid, offset);
            //if multiple buckets, select the most relevant one
            DiamondBucket best_bucket = select_most_relevant_bucket(diamond_buckets);
            if (best_bucket.indices.empty()) {
                std::cout << "[LNS] ERROR: No most relevant bucket found" << std::endl;
                break;
            }
            std::cout << "[LNS] Selected most relevant bucket " << best_bucket.indices[0] 
                    << " with time window: " << best_bucket.earliest_t << " - " << best_bucket.latest_t << ", " 
                    << best_bucket.indices.size() << " conflicts and " 
                    << best_bucket.positions.size() << " positions." << std::endl;


            //Step 7: Solve the best buckets Local Zone
            std::cout << "[LNS] Solving the best bucket Local Zone..." << std::endl;
            //solve the local zone
            LocalZoneResult local_zone_result = solve_local_zone(
                problem.grid, best_bucket, conflict_meta, conflict_map, current_solution, current_max_timesteps, offset);
            

            //Step 8: Update the current solution with the local zone result if found
            if (local_zone_result.solution_found) {
                std::cout << "[LNS] Successfully solved local zone" << std::endl;
                //integrate local zone result into current solution
                current_solution.update_with_local_paths(local_zone_result.local_paths, local_zone_result.local_entry_exit_time);
                //loop back to step 5
            }
            //if impossible to solve, increase makespan
            if (local_zone_result.solution_found == false) {
                std::cout << "[LNS] Impossible to solve local zone with current makespan: " << current_max_timesteps << std::endl;
                conflicts_remain = true;
                break;
            }
        }
        if (!conflicts_remain) {
            std::cout << "[LNS] All conflicts resolved" << std::endl;
            break;
        } else {
            std::cout << "[LNS] Increasing makespan..." << std::endl;
        }
    }
    //Step 9: validate agents paths and return Solution if valid
    bool Valid_Solution = verify_solution_consistency(current_solution.agent_paths, problem.starts, problem.goals, problem.grid);

    if (Valid_Solution) {
        std::cout << "[LNS] Collision-free solution found at makespan " << current_max_timesteps << std::endl;
        std::cout << "[LNS] Final agent paths:" << std::endl;
        SATSolverManager::print_agent_paths(current_solution.agent_paths);
    }else{
        std::cout << "[LNS] ERROR:Solution is not valid" << std::endl;
        SATSolverManager::print_agent_paths(current_solution.agent_paths);
        std::cout << "[LNS] ERROR: Solution is not valid" << std::endl;
    }
    return current_solution.agent_paths;
}