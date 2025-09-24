
#include "../SATSolverManager.h"
#include "lazy_SAT_Solve.h"
#include <unordered_map>
#include <vector>
#include <tuple>
#include <set>
#include <iostream>
#include <utility>




// Helper: Check for vertex collisions for local paths that are within the conflict zone and have different entry and exit times
// Takes local paths with timestep information 
// Returns vector of (agent1, agent2, position, global_timestep) tuples
std::vector<std::tuple<int, int, std::pair<int,int>, int>> check_vertex_collisions_local(
    const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_paths,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int end_t) {
    
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> collisions;
    
    // Find max timesteps in the zone window
    int max_timesteps = end_t - start_t + 1;
    
    // For each timestep in the zone window, check for collisions
    for (int timestep = 0; timestep < max_timesteps; ++timestep) {
        // Map from position to list of agents at that position
        std::unordered_map<std::pair<int,int>, std::vector<int>> position_agents;
        
        // Collect all agents at each position for this timestep
        for (const auto& [agent_id, path] : local_paths) {
            auto entry_exit = local_entry_exit_time.at(agent_id);
            int entry_t = entry_exit.first;
            int exit_t = entry_exit.second;
            
            // Convert zone timestep to global timestep
            int global_timestep = start_t + timestep;
            
            // Check if agent is active at this global timestep
            if (global_timestep >= entry_t && global_timestep <= exit_t) {
                // Calculate the path index for this timestep
                int path_index = global_timestep - entry_t;
                if (path_index < (int)path.size()) {
                    auto position = path[path_index];
                    //find if any other agent is at the same position
                    auto it = position_agents.find(position);
                    if (it != position_agents.end()) {
                        //add collision for each other agent at the same position
                        for (int other_agent : it->second) {
                            collisions.emplace_back(other_agent, agent_id, position, global_timestep);
                        }
                    }
                    //store agent position so if any other agent gets there at the same time we can find the vertex collision
                    position_agents[position].push_back(agent_id);
                }
            }
        }
    }
    
    return collisions;
}

// Helper: Check for edge collisions using SATSolverManager approach
// Takes local paths with timestep information 
// Returns vector of (agent1, agent2, pos1, pos2, global_timestep) tuples
std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> check_edge_collisions_local(
    const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_paths,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int end_t) {
    
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> edge_collisions;
    
    // Find max timesteps in the zone window
    int max_timesteps = end_t - start_t + 1;
    
    // For each timestep (except the last), detect true swaps (opposite edges)
    for (int timestep = 0; timestep < max_timesteps - 1; ++timestep) {
        EdgeAgentMap edge_map;
        edge_map.reserve(local_paths.size());
        
        for (const auto& [agent_id, path] : local_paths) {
            auto entry_exit = local_entry_exit_time.at(agent_id);
            int entry_t = entry_exit.first;
            int exit_t = entry_exit.second;
            
            // Convert zone timesteps to global timesteps
            int global_timestep = start_t + timestep;
            int global_next_timestep = start_t + timestep + 1;
            
            // Check if agent is active at both timesteps
            if (global_timestep >= entry_t && global_timestep <= exit_t &&
                global_next_timestep >= entry_t && global_next_timestep <= exit_t) {
                
                // Calculate path indices for both timesteps
                int path_index = global_timestep - entry_t;
                int next_path_index = global_next_timestep - entry_t;
                
                if (path_index < static_cast<int>(path.size()) && next_path_index < static_cast<int>(path.size())) {
                    // find reverse edge and check if it exists in the edge_map
                    auto from = path[path_index];
                    auto to = path[next_path_index];
                    if (from != to) {
                        auto edge = std::make_pair(from, to);
                        auto rev_edge = std::make_pair(to, from);

                        auto it = edge_map.find(rev_edge);
                        if (it != edge_map.end()) {
                            for (int other_agent : it->second) {
                                //we will find all edge collisions when we check the second agent that is part of the edge collision
                                edge_collisions.emplace_back(other_agent, agent_id,
                                                            rev_edge.first, rev_edge.second,
                                                            global_timestep);
                            }
                        }
                        //store movement of agent so if any other agent moves like that in reverse we can find the edge collision
                        edge_map[edge].push_back(agent_id);
                    }
                }
            }
        }
    }
    
    return edge_collisions;
}

// Helper: Create MDDs with shortest paths + waiting time at goal
// This creates MDDs where agents go to their goal as fast as possible, then wait there
std::vector<std::shared_ptr<MDD>> create_mdds_with_waiting_time(
    const std::vector<std::vector<char>>& grid,
    const std::vector<std::pair<int,int>>& starts,
    const std::vector<std::pair<int,int>>& goals,
    int makespan,
    const std::vector<std::map<std::pair<int,int>, int>>& distance_matrices) {
    
    std::cout << "[SAT] Creating MDDs with shortest paths + waiting time..." << std::endl;
    
    std::vector<std::shared_ptr<MDD>> mdds;
    mdds.reserve(starts.size());
    
    for (size_t agent_id = 0; agent_id < starts.size(); ++agent_id) {
        auto start = starts[agent_id];
        auto goal = goals[agent_id];
        
        // Calculate shortest path length using distance map for this agent
        int shortest_path_length = -1;
        const auto& dist_map = distance_matrices[agent_id];
        auto it = dist_map.find({start.first, start.second});
        if (it != dist_map.end()) {
            shortest_path_length = it->second;
        }
        
        if (shortest_path_length == -1) {
            std::cerr << "[SAT] ERROR: No path found for Agent " << agent_id << std::endl;
            continue;
        }
        
        // Create MDD with shortest path length (inclusive depth)
        // Note: use shortest_path_length directly to ensure sampled paths can reach the goal
        MDDConstructor constructor(grid, start, goal, shortest_path_length);
        auto mdd = constructor.construct_mdd();
        
        if (!mdd) {
            std::cerr << "[SAT] ERROR: Failed to create MDD for Agent " << agent_id << std::endl;
            continue;
        }
        
        mdds.push_back(mdd);
    }
    
    std::cout << "[SAT] Created " << mdds.size() << " MDDs with waiting time structure" << std::endl;
    return mdds;
}


//Solves a local zone with SAT
LazySolveResult lazy_SAT_solve(
    CNF& local_cnf,
    CNFConstructor& cnf_constructor,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int end_t,
    int max_iterations,
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& initial_vertex_collisions,
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& initial_edge_collisions) {

    std::cout << "[SAT] Start solving CNF: " << local_cnf.get_clauses().size() << " clauses and " 
              << (cnf_constructor.get_next_variable_id() - 1) << " variables" << std::endl;


    // Track all discovered collisions during solving without duplicates
    auto set_to_vector_vertex = [](const std::set<std::tuple<int, int, std::pair<int,int>, int>>& s) {
        return std::vector<std::tuple<int, int, std::pair<int,int>, int>>(s.begin(), s.end());
    };
    auto set_to_vector_edge = [](const std::set<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& s) {
        return std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>(s.begin(), s.end());
    };
   
    std::set<std::tuple<int, int, std::pair<int,int>, int>> discovered_vertex_collisions_set;
    std::set<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> discovered_edge_collisions_set;
    // Track latest collisions that were discovered before UNSAT
    std::set<std::tuple<int, int, std::pair<int,int>, int>> latest_discovered_vertex_collisions;
    std::set<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> latest_discovered_edge_collisions;
    // Track discovered collisions for future use
    if (!initial_vertex_collisions.empty()) {
        discovered_vertex_collisions_set.insert(initial_vertex_collisions.begin(), initial_vertex_collisions.end());
        // add them to the cnf
        cnf_constructor.add_collision_clauses_to_cnf(local_cnf, initial_vertex_collisions);
    }
    if (!initial_edge_collisions.empty()) {
        discovered_edge_collisions_set.insert(initial_edge_collisions.begin(), initial_edge_collisions.end());
        // add them to the cnf
        cnf_constructor.add_edge_collision_clauses_to_cnf(local_cnf, initial_edge_collisions);
    }


    bool solution_found = false;
    bool first_iteration = true;
    int iteration = 0;
    std::unordered_map<int, std::vector<std::pair<int,int>>> final_local_paths;
    std::vector<int> initial_assignment;

    //main loop: solve the cnf with minisat
    while (!solution_found && iteration < max_iterations) {
        iteration++;
        std::cout << "[SAT] Solving local zone with SAT iteration " << iteration << "..." << std::endl;
        //solve with minisat
        auto minisat_result = first_iteration
            ? SATSolverManager::solve_cnf_with_minisat(local_cnf)
            : SATSolverManager::solve_cnf_with_minisat(local_cnf, &initial_assignment);
        first_iteration = false;

        if (!minisat_result.satisfiable) {
            std::cout << "[SAT] Local problem is unsatisfiable after " << iteration << " iterations" << std::endl;
            break;
        }
        //we found a solution
        //std::cout << "[SAT] Found solution with " << minisat_result.assignment.size() << " variable assignments" << std::endl;

        // Translate solution to paths 
        auto local_paths = cnf_constructor.cnf_assignment_to_paths(minisat_result.assignment);
        //std::cout << "[SAT] Extracted paths for " << local_paths.size() << " agents" << std::endl;

        // Check paths for collisions have global time step
        auto new_collisions = check_vertex_collisions_local(local_paths, local_entry_exit_time, start_t, end_t);
        auto new_edge_collisions = check_edge_collisions_local(local_paths, local_entry_exit_time, start_t, end_t);
        
        //track discovered collisions for future use
        discovered_vertex_collisions_set.insert(new_collisions.begin(), new_collisions.end());
        discovered_edge_collisions_set.insert(new_edge_collisions.begin(), new_edge_collisions.end());
        //track latest collisions that were discovered before UNSAT (future active conflicts if this run is UNSAT)
        latest_discovered_vertex_collisions.insert(new_collisions.begin(), new_collisions.end());
        latest_discovered_edge_collisions.insert(new_edge_collisions.begin(), new_edge_collisions.end());
        
        std::cout << "[SAT] Found " << new_collisions.size() << " vertex collisions and " 
                  << new_edge_collisions.size() << " edge collisions" << std::endl;
        
        //if no collisions, we have a local solution
        if (new_collisions.empty() && new_edge_collisions.empty()) {
            solution_found = true;
            final_local_paths = std::move(local_paths);
            std::cout << "[SAT] Found collision-free local solution!" << std::endl;
        } else {
            //add new collision clauses to the cnf
            cnf_constructor.add_collision_clauses_to_cnf(local_cnf, new_collisions);
            cnf_constructor.add_edge_collision_clauses_to_cnf(local_cnf, new_edge_collisions);
            //create new partial assignment from the solution and continue
            initial_assignment = minisat_result.assignment; //minisat uses polarity not hard assumptions, we can set all variables and it can still change them

            std::cout << "[SAT] Adding collision clauses and solving again..." << std::endl;
        }
    }
    if (!solution_found) {
        std::cout << "[SAT] Failed to find local solution after " << max_iterations << " iterations" << std::endl;
        std::cout << "[SAT] Local problem appears to be unsatisfiable in current zone" << std::endl;
    }

    
    return {solution_found,
            final_local_paths,
            set_to_vector_vertex(discovered_vertex_collisions_set),
            set_to_vector_edge(discovered_edge_collisions_set),
            set_to_vector_vertex(latest_discovered_vertex_collisions),
            set_to_vector_edge(latest_discovered_edge_collisions)};
        
}