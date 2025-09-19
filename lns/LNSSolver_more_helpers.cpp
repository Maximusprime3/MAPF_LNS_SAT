#include "../SATSolverManager.h"
#include "LNSGeometry.h"
#include "LNSCore.h"
#include "LNSProblemIO.h"
#include <vector>
#include <string>
#include <iostream>
#include <optional>
#include <random>
#include <set>
#include <algorithm>
#include <iterator>
#include <climits>
#include <cmath>
#include <unordered_map>

//TO DO:
//When extending waiting time and finding returning agents
    //we have split mdd one path previously present in the zone the other is the agent returning into the zone
    //currently goes wrong in run_empty48_200_offset1 line 1616

// in the case of 100% coverage, the solver should get all known conflict clauses to add them to the cnf
// MINISAT ASSUMPTIONS ARE NOT HARD, It sets polarity hints via setPolarity, not hard assumptions.

// DONE order conlicts for time priority for solving
// if solved with waiting time used need to redraw all paths

//waiting time strategy maybe can use waiting time for every agent that is in a conflict instead of just one, or one agent for every conflict

//traffic avoidance sampling from mdds
//in der email steht noch was

//cnf problem might be because of mdd miss alignment
//check logic are partly empty mdds ok?
//if not place agents outside of zone (entry point) without any interactions
//THEY are NOT ok, empty layers will produce empty clauses

// Helper: create a masked map keeping only positions within diamond shape walkable
// Uses Manhattan distance: |r - center_row| + |c - center_col| <= radius
static std::vector<std::vector<char>> mask_map_outside_diamond(
    const std::vector<std::vector<char>>& map,
    const std::pair<int,int>& center,
    int radius) {
    if (map.empty() || map[0].empty()) return {};
    int rows = (int)map.size();
    int cols = (int)map[0].size();
    
    // Clamp center to map bounds
    int center_row = std::max(0, std::min(center.first, rows - 1));
    int center_col = std::max(0, std::min(center.second, cols - 1));
    
    // Create full-size map filled with unwalkable '@' and then override diamond region
    std::vector<std::vector<char>> masked(rows, std::vector<char>(cols, '@'));
    
    // Iterate through all positions and check if they're within the diamond
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            // Check if position is inside diamond using Manhattan distance
            if (std::abs(r - center_row) + std::abs(c - center_col) <= radius) {
                masked[r][c] = map[r][c];
            }
        }
    }
    
    return masked;
}




// Helper: create a spatial conflict map for efficient conflict queries
// Returns a 2D array where conflict_map[r][c] = conflict_index if there's a conflict at (r,c), -1 otherwise
static std::vector<std::vector<std::vector<int>>> create_conflict_map(
    const std::vector<std::pair<int,int>>& conflict_points,
    int rows, int cols) {
    std::vector<std::vector<std::vector<int>>> conflict_map(rows, std::vector<std::vector<int>>(cols));
    
    for (size_t i = 0; i < conflict_points.size(); ++i) {
        auto [r, c] = conflict_points[i];
        if (r >= 0 && r < rows && c >= 0 && c < cols) {
            conflict_map[r][c].push_back(static_cast<int>(i));
        }
    }
    
    return conflict_map;
}




// Helper: convert raw vertex and edge collisions into conflict metadata and
// conflict points used for bucket construction.
static void collect_conflicts(
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& vertex_collisions,
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& edge_collisions,
    std::vector<std::pair<int,int>>& conflict_points,
    std::vector<ConflictMeta>& conflict_meta) {
    conflict_points.clear();
    conflict_meta.clear();
    conflict_points.reserve(vertex_collisions.size() + edge_collisions.size() * 2);
    conflict_meta.reserve(vertex_collisions.size() + edge_collisions.size() * 2);

    // Vertex conflicts contribute a single point and metadata entry.
    for (const auto& v : vertex_collisions) {
        int a1 = std::get<0>(v);
        int a2 = std::get<1>(v);
        auto pos = std::get<2>(v);
        int t = std::get<3>(v);
        conflict_points.push_back(pos);
        conflict_meta.push_back(ConflictMeta{a1, a2, t, false, pos, {-1, -1}});
    }

    // Edge conflicts contribute both end points to improve spatial clustering.
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
}

static void collect_conflicts_meta(
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& vertex_collisions,
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& edge_collisions,
    std::vector<ConflictMeta>& conflict_meta) {
    conflict_meta.clear();
    conflict_meta.reserve(vertex_collisions.size() + edge_collisions.size());

    // Vertex conflicts contribute a single metadata entry.
    for (const auto& v : vertex_collisions) {
        int a1 = std::get<0>(v);
        int a2 = std::get<1>(v);
        auto pos = std::get<2>(v);
        int t = std::get<3>(v);
        conflict_meta.push_back(ConflictMeta{a1, a2, t, false, pos, {-1, -1}});
    }

    // Edge conflicts contribute a single metadata entry.
    for (const auto& e : edge_collisions) {
        int a1 = std::get<0>(e);
        int a2 = std::get<1>(e);
        auto pos1 = std::get<2>(e);
        auto pos2 = std::get<3>(e);
        int t = std::get<4>(e);
        conflict_meta.push_back(ConflictMeta{a1, a2, t, true, pos1, pos2});
    }
}

// Helper: form diamond shaped buckets around conflicts.  Each conflict belongs
// to exactly one bucket.  The algorithm incrementally grows a diamond around a
// seed conflict, merging nearby conflicts that fall into the diamond.
static std::vector<DiamondBucket> build_diamond_buckets(
    const std::vector<std::pair<int,int>>& conflict_points,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<std::vector<char>>& map,
    const std::set<int>& solved_conflict_indices,
    const std::vector<ConflictMeta>& conflict_meta,
    int offset) {

    std::vector<DiamondBucket> diamond_buckets;
    std::vector<char> diamond_used(conflict_points.size(), 0);
    // Dimensions for safe conflict_map access
    const int rows = (int)conflict_map.size();
    const int cols = rows > 0 ? (int)conflict_map[0].size() : 0;

    for (size_t i = 0; i < conflict_points.size(); ++i) {
        if (diamond_used[i] || solved_conflict_indices.count(i)) continue;
        std::cout << "[LNS] Building diamond bucket for conflict " << i << std::endl;
        std::vector<std::pair<int,int>> bucket_conflicts;
        bucket_conflicts.push_back(conflict_points[i]);
        // Collect conflict indices as we grow the bucket to ensure buckets always carry conflicts
        std::set<int> index_set;
        index_set.insert((int)i);
        // Track earliest timestep incrementally
        int bucket_earliest_t = conflict_meta[i].timestep; //set to timestep of first conflict

        

        std::set<std::pair<int,int>> previous_shape;
        bool found_new_conflicts = true;
        while (found_new_conflicts) {
            found_new_conflicts = false;
            // Use conflict_map dimensions implicitly via map overloads
            auto current_shape = create_shape_from_conflicts(bucket_conflicts, offset, map);
            auto new_positions = find_new_positions(current_shape, previous_shape, map);

            for (const auto& pos : new_positions) {
                auto [r, c] = pos;
                // Bounds guard to avoid OOB access
                if (r < 0 || r >= rows || c < 0 || c >= cols) {
                    std::cerr << "[ERROR] Found position (" << r << "," << c << ") outside of map bounds" << std::endl;
                    continue;
                }
                if (conflict_map[r][c].empty()) continue;
                for (int conflict_idx : conflict_map[r][c]) {
                    // Index validity guard
                    if (conflict_idx < 0 || conflict_idx >= (int)conflict_points.size()) {
                        std::cerr << "[ERROR] Found invalid conflict index " << conflict_idx << std::endl;
                        continue;
                    }
                    if (diamond_used[conflict_idx]) {
                        std::cerr << "[ERROR] Found already used conflict " << conflict_idx
                                  << " at position (" << r << "," << c
                                  << ") in new positions of diamond bucket." << std::endl;
                        continue;
                    }
                    if (!solved_conflict_indices.count(conflict_idx)) {
                        bucket_conflicts.push_back(conflict_points[conflict_idx]);
                        diamond_used[conflict_idx] = 1;
                        found_new_conflicts = true;
                        index_set.insert(conflict_idx);
                        if (conflict_idx >= 0 && conflict_idx < (int)conflict_meta.size()) {
                            bucket_earliest_t = std::min(bucket_earliest_t, conflict_meta[conflict_idx].timestep); //update earliest timestep of the bucket
                        }
                    }
                }
            }

            previous_shape = std::move(current_shape);
        }

        auto diamond_positions = create_shape_from_conflicts(bucket_conflicts, offset, map);
        DiamondBucket diamond_bucket;
        diamond_bucket.positions = std::move(diamond_positions);
        
        diamond_bucket.indices.assign(index_set.begin(), index_set.end());
        diamond_bucket.earliest_t = bucket_earliest_t;

        diamond_buckets.push_back(std::move(diamond_bucket));
    }

    return diamond_buckets;
}


// Helper: Check for vertex collisions for local paths that are within the conflict zone and have different entry and exit times
// Takes local paths with timestep information and mimics SATSolverManager::find_vertex_collisions
// Returns vector of (agent1, agent2, position, timestep) tuples
static std::vector<std::tuple<int, int, std::pair<int,int>, int>> check_vertex_collisions_local(
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
                            collisions.emplace_back(other_agent, agent_id, position, timestep);
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
// Takes local paths with timestep information and mimics SATSolverManager::find_edge_collisions
// Returns vector of (agent1, agent2, pos1, pos2, timestep) tuples
static std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> check_edge_collisions_local(
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
                                                            timestep);
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

// Helper: Align an MDD to a specific time window without introducing placeholder empty levels.
// Only actual agent levels are retained and shifted into the target window,
//  keeping downstream CNF construction focused on levels that contain nodes.
static void align_mdd_to_time_window(std::shared_ptr<MDD> mdd,
                                    int entry_t, int exit_t, //agent's entry and exit times
                                    int start_t, int end_t) { //time window of problem zone start and end
   

    if (!mdd) { // do we have an mdd?
        return;
    }

    if (end_t < start_t) { // does start and end make sense?
        std::cerr << "[LNS] ERROR: Invalid time window for MDD alignment (end_t < start_t)." << std::endl;
        mdd->levels.clear();
        return;
    }
    
    int zone_mdd_length = end_t - start_t + 1;
    // Calculate relative timesteps within the time window
    int relative_entry = entry_t - start_t;  // 0-based within the time window
    int relative_exit = exit_t - start_t;    // 0-based within the time window
    
    relative_entry = std::max(0, std::min(relative_entry, zone_mdd_length - 1));
    //relative_exit = std::max(relative_entry, std::min(relative_exit, zone_mdd_length - 1)); //we don't need to do this as we don't add empty levels after the agent's exit

    auto original_levels = mdd->levels;
    std::map<int, std::vector<std::shared_ptr<MDDNode>>> aligned_levels;

    //for (int level = 0; level < relative_entry; ++level) {//add empty levels before the agent's entry
    //    aligned_levels[level] = {};
    //}

    
    // Shift the agent's MDD levels to the correct position
    //int last_active_level = relative_entry - 1;
    for (const auto& [level, nodes] : original_levels) {
        int new_level = level + relative_entry + start_t; //shift the level to the correct position in the window
        if (new_level >= zone_mdd_length + start_t) {
            std::cerr << "[LNS] WARNING: MDD level " << new_level
                      << " exceeds time window length " << zone_mdd_length + start_t
                      << "; truncating." << std::endl;
            continue;
        }

        auto& target_nodes = aligned_levels[new_level]; //get the target nodes
        target_nodes = nodes;
        for (auto& node : target_nodes) {
            node->time_step = new_level; //MDD is now in absolute time scale similar to collision metadata and update logic 
        }
        //last_active_level = std::max(last_active_level, new_level);
    }


    //last_active_level = std::max(last_active_level, relative_exit);

    //for (int level = last_active_level + 1; level < zone_mdd_length; ++level) { //add empty levels after the last active level
    //    aligned_levels[level] = {};
    //}

    mdd->levels = std::move(aligned_levels);
}

// Structure to hold lazy solving results including discovered collisions
struct LazySolveResult {
    bool solution_found;
    std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths;
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> discovered_vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> discovered_edge_collisions;
};

// Helper: Lazy solving of conflict zone
// Returns LazySolveResult with solution status, paths, and discovered collisions
static LazySolveResult 
lazy_solve_conflict_zone(CNF& local_cnf, 
                        CNFConstructor& cnf_constructor,
                        const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
                        int start_t, int end_t,
                        int max_iterations = 100000,
                        // Optional: pre-discovered collisions to avoid rediscovering
                        const std::vector<std::tuple<int, int, std::pair<int,int>, int>>* known_vertex_collisions = nullptr,
                        const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>* known_edge_collisions = nullptr) {

    std::cout << "[LNS] Starting lazy solving of conflict zone..." << std::endl;

    bool local_solution_found = false;
    int iteration = 0;
    std::unordered_map<int, std::vector<std::pair<int,int>>> final_local_paths;

    // Track all discovered collisions during solving
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> discovered_vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> discovered_edge_collisions;
    // Track discovered collisions for future use
    if (known_vertex_collisions) {
        discovered_vertex_collisions.insert(discovered_vertex_collisions.end(), known_vertex_collisions->begin(), known_vertex_collisions->end());
    }
    if (known_edge_collisions) {
        discovered_edge_collisions.insert(discovered_edge_collisions.end(), known_edge_collisions->begin(), known_edge_collisions->end());
    }

    // Add pre-discovered collisions to avoid rediscovering them
    if (known_vertex_collisions && !known_vertex_collisions->empty()) {
        std::cout << "[LNS] Adding " << known_vertex_collisions->size() << " pre-discovered vertex collisions..." << std::endl;
        for (const auto& collision : *known_vertex_collisions) {
            int agent1, agent2, timestep;
            std::pair<int,int> pos;
            std::tie(agent1, agent2, pos, timestep) = collision;
            try {
                std::vector<int> clause = cnf_constructor.add_single_collision_clause(
                    agent1, agent2, pos, timestep, false);
                if (!clause.empty()) local_cnf.add_clause(clause);
            } catch (const std::exception& e) {
                std::cout << "[LNS] Skipping invalid pre-discovered vertex collision (" << agent1 << "," << agent2
                          << ") at pos (" << pos.first << "," << pos.second << ") t=" << timestep
                          << ": " << e.what() << std::endl;
            }
        }
    }
    // there can be duplicates in the known_edge_collisions, so we need to check for them
    std::set<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> known_edge_collisions_set;

    if (known_edge_collisions && !known_edge_collisions->empty()) {
        std::cout << "[LNS] Adding " << known_edge_collisions->size() << " pre-discovered edge collisions..." << std::endl;
        //test print all known edge collisions
        std::cout << "[LNS] Known edge collisions: " << std::endl;

        for (const auto& edge_collision : *known_edge_collisions) {
            //check if we already added this edge collision
            if (known_edge_collisions_set.count(edge_collision) > 0) {
                continue;
            }
            known_edge_collisions_set.insert(edge_collision);
            int agent1, agent2, timestep;
            std::pair<int,int> pos1, pos2;
            std::tie(agent1, agent2, pos1, pos2, timestep) = edge_collision;
            //print adding edge collision
            std::cout << "[LNS] Adding edge collision: " << agent1 << ", " << agent2
                      << ", (" << pos1.first << "," << pos1.second << ")"
                      << " <-> (" << pos2.first << "," << pos2.second << ")"
                      << ", t=" << timestep << std::endl;
            try {
                std::vector<int> clause = cnf_constructor.add_single_edge_collision_clause(
                    agent1, agent2, pos1, pos2, timestep, false);
                if (!clause.empty()) local_cnf.add_clause(clause);
            } catch (const std::exception& e) {
                std::cout << "[LNS] Skipping invalid pre-discovered edge collision (" << agent1 << "," << agent2
                          << ") pos1=(" << pos1.first << "," << pos1.second << ") pos2=(" << pos2.first << "," << pos2.second
                          << ") t=" << timestep << ": " << e.what() << std::endl;
            }
        }
    }

    //create empty assignment, which we can later fill with the previous solution
    std::vector<int> initial_assignment;
    bool first_iteration = true;
    bool last_iteration_was_satisfiable = true;
    while (!local_solution_found && iteration < max_iterations) {
        iteration++;
        std::cout << "[LNS] Lazy solving iteration " << iteration << "..." << std::endl;

        // Solve the current CNF with MiniSAT
        //solve without assignment if first iteration or last iteration was unsatisfiable
        MiniSatSolution minisat_result;
        if (first_iteration || !last_iteration_was_satisfiable) {
            
            minisat_result = SATSolverManager::solve_cnf_with_minisat(local_cnf);
            first_iteration = false;
        } else {
            
            minisat_result = SATSolverManager::solve_cnf_with_minisat(local_cnf, &initial_assignment);
        }

        if (!minisat_result.satisfiable) {
            std::cout << "[LNS] Local problem is unsatisfiable after " << iteration << " iterations" << std::endl;
        //try solving without initial assignment once(set flag), only when this statement is reached twice in a row, we know there is no solution
            if (last_iteration_was_satisfiable) {
                last_iteration_was_satisfiable = false; //trying to solve without assumptions will show if there is a solution at all
                //safety run solving without assignment
                std::cout << "[LNS] No Solution found -> Safety run solving without assignment" << std::endl;
                continue;
            } else {
                //safety run failed
                std::cout << "[LNS] No Solution found + Safety run failed -> UNSAT" << std::endl;
                break;
            }
        }

        std::cout << "[LNS] Found solution with " << minisat_result.assignment.size() << " variable assignments" << std::endl;

        // Translate solution to paths using helper function
        auto local_paths = cnf_constructor.cnf_assignment_to_paths(minisat_result.assignment);
        std::cout << "[LNS] Extracted paths for " << local_paths.size() << " agents" << std::endl;

        // Check paths for collisions using SATSolverManager approach
        auto new_collisions = check_vertex_collisions_local(local_paths, local_entry_exit_time, start_t, end_t);
        auto new_edge_collisions = check_edge_collisions_local(local_paths, local_entry_exit_time, start_t, end_t);
        //adjust timestep of local collisions to be global time step
        //print adjusting collisions by start_t
        std::cout << "[LNS] Adjusting collisions by start_t: " << start_t << std::endl;
        for (auto& collision : new_collisions) {
            std::get<3>(collision) += start_t;
        }
        for (auto& collision : new_edge_collisions) {
            std::get<4>(collision) += start_t;
        }

        // Track discovered collisions for future use
        discovered_vertex_collisions.insert(discovered_vertex_collisions.end(), new_collisions.begin(), new_collisions.end());
        discovered_edge_collisions.insert(discovered_edge_collisions.end(), new_edge_collisions.begin(), new_edge_collisions.end());

        std::cout << "[LNS] Found " << new_collisions.size() << " vertex collisions and " 
        << new_edge_collisions.size() << " edge collisions" << std::endl;

        // If no collisions, we have a local solution
        if (new_collisions.empty() && new_edge_collisions.empty()) {
            local_solution_found = true;
            final_local_paths = std::move(local_paths);
            std::cout << "[LNS] Found collision-free local solution!" << std::endl;
        } else {
            // Add collision clauses, create new partial(leaving out all paths of colliding agents) assignment from the solution and continue
            initial_assignment = cnf_constructor.partial_assignment_from_paths(local_paths);
            //set flag to true, because we found a solution
            last_iteration_was_satisfiable = true;

            std::cout << "[LNS] Adding collision clauses and solving again..." << std::endl;

            // Add vertex collision clauses
            for (const auto& collision : new_collisions) {
                int agent1, agent2, timestep;
                std::pair<int,int> pos;
                std::tie(agent1, agent2, pos, timestep) = collision;
                //print the vertex collision
                std::cout << "  Vertex collision: " << agent1 << ", " << agent2
                          << ", (" << pos.first << "," << pos.second << ")"
                          << ", t=" << timestep << std::endl;

                std::vector<int> clause = cnf_constructor.add_single_collision_clause(
                agent1, agent2, pos, timestep, false);
                local_cnf.add_clause(clause);
            }

            // Add edge collision clauses
            for (const auto& edge_collision : new_edge_collisions) {
                int agent1, agent2, timestep;
                std::pair<int,int> pos1, pos2;
                std::tie(agent1, agent2, pos1, pos2, timestep) = edge_collision;
                //print the edge collision
                std::cout << "  Edge collision: " << agent1 << ", " << agent2
                          << ", (" << pos1.first << "," << pos1.second << ")"
                          << " <-> (" << pos2.first << "," << pos2.second << ")"
                          << ", t=" << timestep << std::endl;

                std::vector<int> clause = cnf_constructor.add_single_edge_collision_clause(
                agent1, agent2, pos1, pos2, timestep, false);
                local_cnf.add_clause(clause);
            }

        }
    }

        if (!local_solution_found) {
            std::cout << "[LNS] Failed to find local solution after " << max_iterations << " iterations" << std::endl;
            std::cout << "[LNS] Local problem appears to be unsatisfiable in current zone" << std::endl;
    }

        return {local_solution_found, final_local_paths, discovered_vertex_collisions, discovered_edge_collisions};
}


// Helper: Lazy solving with waiting time strategy
// Returns LazySolveResult with solution status, paths, and discovered collisions
static LazySolveResult 
lazy_solve_with_waiting_time(CurrentSolution& current_solution,
                            const std::vector<std::vector<char>>& masked_map,
                            const std::set<std::pair<int,int>>& zone_positions_set,
                            std::unordered_map<int, std::shared_ptr<MDD>>& local_mdds,
                            std::unordered_map<int, std::vector<std::pair<int,int>>>& local_zone_paths,
                            std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
                            const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& initial_vertex_collisions,
                            const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& initial_edge_collisions,
                            const std::vector<ConflictMeta>& conflict_meta,
                            const std::vector<int>& best_bucket_indices,
                            int start_t, int end_t) {
    
    std::cout << "[LNS] Starting waiting time strategy..." << std::endl;
    
    // Step 1: Backup current waiting times
    auto waiting_time_backup = current_solution.backup_waiting_times();
    // Also snapshot original entry/exit times to compute expansion deltas later
    std::unordered_map<int, std::pair<int,int>> original_entry_exit_time = local_entry_exit_time;
    
    // Step 2: find conflicts and determine agents to use waiting time for
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> all_current_vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> all_current_edge_collisions;
    std::vector<ConflictMeta> all_current_conflicts;
    std::set<int> chosen_agents_currently_in_conflicts;
    //get conflicts
    collect_conflicts_meta(initial_vertex_collisions, initial_edge_collisions, all_current_conflicts); //resets all_current_conflicts
    //helper function to iterate through all current conflicts -> add agents to chosen_agents_currently_in_conflicts
    //per conflict check if one of the agents already is in the set, if not add the one with more waiting time
    auto choose_agents_from_current_conflicts = [&](const std::vector<ConflictMeta>& all_current_conflicts,
                                                    std::set<int>& chosen_agents_currently_in_conflicts) -> bool {
        //reset chosen_agents_currently_in_conflicts
        chosen_agents_currently_in_conflicts.clear();
        bool can_use_waiting_time_for_all_current_conflicts = true;
        //iterate through all current conflicts -> skip if one of the agents is already chosen, otherwise add the one with more waiting time
        for (const auto& conflict : all_current_conflicts) {
            int agent1 = conflict.agent1;
            int agent2 = conflict.agent2;
            //check if one of the agents is already chosen
            if (chosen_agents_currently_in_conflicts.count(agent1) > 0 || chosen_agents_currently_in_conflicts.count(agent2) > 0) {
                continue;
            }
            
            int agent1_waiting = current_solution.get_waiting_time(agent1);
            int agent2_waiting = current_solution.get_waiting_time(agent2);
            
            //if both agents have 0 waiting time, we can't use waiting time for all conflicts
            if (agent1_waiting == 0 && agent2_waiting == 0) {
                can_use_waiting_time_for_all_current_conflicts = false;
                chosen_agents_currently_in_conflicts.clear();
                break;
            }
            int selected_agent = (agent1_waiting >= agent2_waiting) ? agent1 : agent2;
            chosen_agents_currently_in_conflicts.insert(selected_agent); //add the one with more waiting time
        }
        return can_use_waiting_time_for_all_current_conflicts;
    };
    //choose agents
    bool can_use_waiting_time_for_all_current_conflicts = choose_agents_from_current_conflicts(all_current_conflicts, chosen_agents_currently_in_conflicts);

    if (can_use_waiting_time_for_all_current_conflicts) {
        //print all chosen agents involved in conflicts
        std::cout << "[LNS] Chosen agents involved in conflicts: ";
        for (int agent_id : chosen_agents_currently_in_conflicts) {
            std::cout << agent_id << " ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "[LNS] Not enough waiting time. Can't use waiting time for all current conflicts" << std::endl;
        return {false, {}, initial_vertex_collisions, initial_edge_collisions};
    }
    
    
    // We have waiting time available - use it to resolve conflicts
    std::cout << "[LNS] Found " << chosen_agents_currently_in_conflicts.size() 
              << " agents in conflicts with, all with waiting time:" << std::endl;

    
    // Step 3: Use waiting time to expand MDDs and resolve conflicts
    // Strategy: One agent per conflict, selected based on waiting time
    bool waiting_time_solution_found = false;
    int attempt = 0;
    std::set<int> exhausted_agents;
    //zone start never changes
    auto zone_start_t = start_t;
    // Persistent collision tracking across all waiting time attempts
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> all_discovered_vertex_collisions = initial_vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> all_discovered_edge_collisions = initial_edge_collisions;
    
    
    while (!waiting_time_solution_found) {
        attempt++;
        std::cout << "[LNS] Waiting time attempt " << attempt << "..." << std::endl;
        //We have all_current_conflicts either from initial or from previous attempts, we try to use waiting time for each conflict
        bool can_use_waiting_time_for_all_current_conflicts = choose_agents_from_current_conflicts(all_current_conflicts, chosen_agents_currently_in_conflicts);

        if (!can_use_waiting_time_for_all_current_conflicts) {
            std::cout << "[LNS] No more waiting time available for conflict resolution" << std::endl;
            break;
        }

        std::cout << "[LNS] Will expand MDD for " << chosen_agents_currently_in_conflicts.size() << " chosen agents involved in " << all_current_conflicts.size() << " conflicts" << std::endl;
        
        // track the furthest time window end across all expansions in this attempt
        int new_end_t = end_t;

        //iterate through all chosen agents to expand MDDs
        bool extended_time_window = false;
        for (int agent_id : chosen_agents_currently_in_conflicts) {
            //get entry/exit time
            auto entry_exit_it = local_entry_exit_time.find(agent_id);
            if (entry_exit_it == local_entry_exit_time.end()) {
                std::cerr << "[LNS] ERROR: Missing entry/exit info for agent " << agent_id << std::endl;
                continue;
            }
            int entry_t = entry_exit_it->second.first;
            int exit_t = entry_exit_it->second.second;

            //get path
            auto path_it = local_zone_paths.find(agent_id);
            if (path_it == local_zone_paths.end() || path_it->second.empty()) {
                std::cerr << "[LNS] ERROR: Missing local zone path for agent " << agent_id << std::endl;
                continue;
            }
            const auto& local_path = path_it->second;

            //reduce waiting time by 1
            current_solution.use_waiting_time(agent_id, 1);

            //expand MDD by 1 timestep
            std::cout << "[LNS] Expanding MDD for agent " << agent_id << " by 1 timestep" << std::endl; 

            //get zone start and goal positions
            auto zone_start_pos = local_path.front();
            auto zone_goal_pos = local_path.back();

            //calculate new path length
            int original_path_length = exit_t - entry_t + 1;
            int new_path_length = original_path_length + 1;
            int new_exit_t = exit_t + 1;
           
            if (new_exit_t > new_end_t) {
                new_end_t = new_exit_t;
                extended_time_window = true;
            }

            //create new MDD
            MDDConstructor constructor(masked_map, zone_start_pos, zone_goal_pos, new_path_length - 1);
            auto expanded_mdd = constructor.construct_mdd();

            //align MDD to the time window
            align_mdd_to_time_window(expanded_mdd, entry_t, new_exit_t, start_t, new_end_t);

            //update local MDD
            local_mdds[agent_id] = expanded_mdd;

            //update entry/exit time
            local_entry_exit_time[agent_id] = {entry_t, new_exit_t};

            std::cout << "[LNS] Updated agent " << agent_id << " entry/exit time: [" << entry_t << ", " << new_exit_t << "]" << std::endl;
            
        }

        //determine if we need to extend the time window
        if (extended_time_window) {
            std::cout << "[LNS] Time window extended: [" << start_t << ", " << end_t << "] -> [" << start_t << ", " << new_end_t << "]" << std::endl;
            
            // Check for new agents that enter the zone at the new timestep and could collide with agents in the local zone
            std::set<int> new_agents_in_extended_window;
            for (const auto& rc : zone_positions_set) {
                int r = rc.first, c = rc.second;
                // Since we expand by exactly 1 timestep, just check the new timestep
                auto here = current_solution.get_agents_at_position_time(r, c, new_end_t);
                new_agents_in_extended_window.insert(here.begin(), here.end());
            }
            if (!new_agents_in_extended_window.empty()) {
                std::cout << "[LNS] Found " << new_agents_in_extended_window.size() 
                          << " new agents in extended time window: ";
                for (int agent_id : new_agents_in_extended_window) {
                    std::cout << agent_id << " ";
                }
                std::cout << std::endl;
            }

            //print local zone agents
            std::cout << "[LNS] Local zone agents: "; 
            for (const auto& [agent_id, path] : local_zone_paths) {
                std::cout << agent_id << " ";
            }
            std::cout << std::endl;

            // print if new agents are already in the local zone
            std::cout << "[LNS] New agents already in local zone: ";
            for (int agent_id : new_agents_in_extended_window) {
                if (local_zone_paths.find(agent_id) != local_zone_paths.end()) {
                    std::cout << agent_id << " ";
                }
            }
            std::cout << std::endl;
            
            // Add new agents to the local problem
            for (int new_agent_id : new_agents_in_extended_window) {
                //skip if member of to be expanded agents because we already expanded them 
                if (chosen_agents_currently_in_conflicts.count(new_agent_id) > 0) {
                    continue;
                }
                if (local_zone_paths.find(new_agent_id) == local_zone_paths.end()) { //this checks if the new agent wasnt in the local zone before
                    // Extract local path for this new agent
                    const auto& path = current_solution.agent_paths.at(new_agent_id);
                    std::vector<std::pair<int,int>> segment;
                    int entry_t = -1, exit_t = -1;
                    
                    // Since this agent wasn't in the zone before, they can only enter at the new timestep
                    // Check if agent is in zone at the new timestep
                    if (new_end_t < (int)path.size()) {
                        const auto& new_pos = path[new_end_t];
                        if (zone_positions_set.count(new_pos)) {
                            entry_t = new_end_t;
                            exit_t = new_end_t;
                            segment.push_back(new_pos);
                        }
                    }
                    if (!segment.empty()) {
                        local_zone_paths[new_agent_id] = std::move(segment);
                        local_entry_exit_time[new_agent_id] = {entry_t, exit_t};
                        std::cout << "  Added new agent " << new_agent_id << " local segment t=[" << entry_t << "," << exit_t
                                  << "] len=" << (exit_t - entry_t + 1) << std::endl;
                        
                    
                        // Create MDD for the new agent
                        auto zone_start_pos = segment[0];
                        auto zone_goal_pos = segment[0];
                        int agent_path_length = exit_t - entry_t + 1;
                        MDDConstructor constructor(masked_map, zone_start_pos, zone_goal_pos, agent_path_length - 1);
                        auto agent_mdd = constructor.construct_mdd();
                        
                        // Align MDD to the extended time window
                        align_mdd_to_time_window(agent_mdd, entry_t, exit_t, start_t, new_end_t);
                        
                        local_mdds[new_agent_id] = agent_mdd;
                        std::cout << "  Created MDD for new agent " << new_agent_id << std::endl;
                        
                    } else {std::cout << "[LNS] ERROR: No segment found for new agent " << new_agent_id << std::endl;}
                } else {//agent is a returning agent. The whole first visit is already in the local path

                    //check if the agent was present at the previous last timestep
                    if (local_entry_exit_time[new_agent_id].second == new_end_t - 1) {//agent is continuing
                        std::cout << "[LNS] New agent " << new_agent_id << " was present at the previous last timestep" << std::endl;
                        std::cout << "[LNS] New agent " << new_agent_id << " is a continuing agent" << std::endl;

                        
                        //update exit time and entry time
                        entry_t = local_entry_exit_time[new_agent_id].first;
                        exit_t = new_end_t;
                        local_entry_exit_time[new_agent_id] = {entry_t, exit_t};
                        std::cout << "[LNS] New agent " << new_agent_id << " local segment t=[" << entry_t << "," << exit_t
                                  << "] len=" << (exit_t - entry_t + 1) << std::endl;
                        
                        //rebuild MDD for the new agent with the new exit time and position
                        auto zone_start_pos = local_zone_paths[new_agent_id].front();
                        const auto& agent_path = current_solution.agent_paths.at(new_agent_id);
                        auto zone_goal_pos = agent_path[exit_t];
                        int agent_path_length = exit_t - start_t + 1;
                        MDDConstructor constructor(masked_map, zone_start_pos, zone_goal_pos, agent_path_length - 1);
                        auto agent_mdd = constructor.construct_mdd();
                        align_mdd_to_time_window(agent_mdd, entry_t, exit_t , start_t, new_end_t);
                        local_mdds[new_agent_id] = agent_mdd;
                        std::cout << "[LNS] Created MDD for new agent " << new_agent_id << std::endl;
                        
                    } else {
                        std::cout << "[LNS] New agent " << new_agent_id << " was not present at the previous last timestep" << std::endl;
                        std::cout << "[LNS] New agent " << new_agent_id << " is a returning agent" << std::endl;
                        //print agent entry exit time
                        std::cout << "[LNS] New agent " << new_agent_id << " entry/exit time: [" << local_entry_exit_time[new_agent_id].first << "," << local_entry_exit_time[new_agent_id].second << "]" << std::endl;
                        //compare end time with new_end_t
                        if (local_entry_exit_time[new_agent_id].second == new_end_t -1) {
                            std::cout << "[LNS] New agent " << new_agent_id << " end time is the same as new_end_t - 1" << std::endl;
                            std::cout << "THIS SHOULD NOT HAPPEN" << std::endl;
                        } else {
                            std::cout << "[LNS] New agent " << new_agent_id << " end time is not the same as new_end_t - 1" << std::endl;
                        }
                        //print new_end_t
                        std::cout << "[LNS] New agent " << new_agent_id << " new_end_t: " << new_end_t << std::endl;
                        
                        
                        //build second MDD for the new agent with just the returning part (len = 1)
                        //get position from global path
                        const auto& agent_path = current_solution.agent_paths.at(new_agent_id);
                        auto zone_start_pos = agent_path[new_end_t];
                        auto zone_goal_pos = agent_path[new_end_t];
                        int agent_path_length = 1;
                        MDDConstructor constructor(masked_map, zone_start_pos, zone_goal_pos, agent_path_length - 1);
                        auto agent_mdd = constructor.construct_mdd();
                        entry_t = new_end_t;
                        align_mdd_to_time_window(agent_mdd, entry_t, new_end_t, start_t, new_end_t);
                        //append to the existing MDD
                        local_mdds[new_agent_id]->levels[new_end_t] = {agent_mdd->levels[new_end_t]};
                        std::cout << "[LNS] Appended new agent " << new_agent_id << " MDD for timestep " << new_end_t << std::endl;
                    }
                   
                }
            }
        } else {
            std::cout << "[LNS] Time window unchanged: [" << start_t << ", " << end_t << "] " << std::endl;
        }
        
        
        // Recreate local CNF with expanded MDDs
        std::cout << "[LNS] Recreating local CNF with expanded MDDs" << std::endl;
        
        // Create new CNF constructor with updated MDDs
        CNFConstructor new_cnf_constructor(local_mdds, true); // true = lazy encoding
        CNF new_local_cnf = new_cnf_constructor.construct_cnf();
        
        // Note: Collision clauses will be added by lazy_solve_conflict_zone() using all_discovered_*_collisions
        
        // Try lazy solving with expanded MDD, using all previously discovered collisions
        std::cout << "[LNS] Attempting lazy solving with expanded MDD..." << std::endl;
        auto using_waiting_time_result = lazy_solve_conflict_zone(
            new_local_cnf, new_cnf_constructor, local_entry_exit_time, start_t, new_end_t,
            10000, // max_iterations
            &all_discovered_vertex_collisions, // Use all previously discovered collisions
            &all_discovered_edge_collisions);
        
        if (using_waiting_time_result.solution_found) {
            waiting_time_solution_found = true;
            std::cout << "[LNS] Successfully resolved conflicts using waiting time strategy!" << std::endl;
            
            // Update global solution with expanded local paths using waiting-time aware integration
            current_solution.update_with_local_paths_waiting(
                using_waiting_time_result.local_paths,
                original_entry_exit_time,
                local_entry_exit_time); 

            return LazySolveResult{true, using_waiting_time_result.local_paths, all_discovered_vertex_collisions, all_discovered_edge_collisions};
        } else { //no solution found 
            std::cout << "[LNS] Expanded MDD did not resolve conflicts, trying next agent..." << std::endl;
            
            // Update persistent collision tracking with newly discovered collisions
            all_discovered_vertex_collisions.insert(all_discovered_vertex_collisions.end(), 
                                                  using_waiting_time_result.discovered_vertex_collisions.begin(),
                                                  using_waiting_time_result.discovered_vertex_collisions.end());
            all_discovered_edge_collisions.insert(all_discovered_edge_collisions.end(),
                                                using_waiting_time_result.discovered_edge_collisions.begin(),
                                                using_waiting_time_result.discovered_edge_collisions.end());

            //update all_current_conflicts
            collect_conflicts_meta(
                using_waiting_time_result.discovered_vertex_collisions, 
                using_waiting_time_result.discovered_edge_collisions, 
                all_current_conflicts);
            
            std::cout << "[LNS] Updated collision tracking: " << all_discovered_vertex_collisions.size() 
                      << " vertex collisions, " << all_discovered_edge_collisions.size() << " edge collisions" << std::endl;
        }
    }
    
    if (!waiting_time_solution_found) {
        std::cout << "[LNS] Waiting time strategies exhausted, restoring backup..." << std::endl;
        // Restore waiting times if no solution found
        current_solution.restore_waiting_times(waiting_time_backup);
        //dont need to restore entry exit time because next we will expand the zone and they will change anyways
    }
    
    return {false, {}, all_discovered_vertex_collisions, all_discovered_edge_collisions};
}



// Helper: Create MDDs with shortest paths + waiting time at goal
// This creates MDDs where agents go to their goal as fast as possible, then wait there
static std::vector<std::shared_ptr<MDD>> create_mdds_with_waiting_time(
    const std::vector<std::vector<char>>& grid,
    const std::vector<std::pair<int,int>>& starts,
    const std::vector<std::pair<int,int>>& goals,
    int makespan,
    const std::vector<std::map<std::pair<int,int>, int>>& distance_matrices) {
    
    std::cout << "[LNS] Creating MDDs with shortest paths + waiting time..." << std::endl;
    
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
            std::cerr << "[ERROR] No path found for agent " << agent_id << std::endl;
            continue;
        }
        
        // Create MDD with shortest path length (inclusive depth)
        // Note: use shortest_path_length directly to ensure sampled paths can reach the goal
        MDDConstructor constructor(grid, start, goal, shortest_path_length);
        auto mdd = constructor.construct_mdd();
        
        if (!mdd) {
            std::cerr << "[ERROR] Failed to create MDD for agent " << agent_id << std::endl;
            continue;
        }
        
        // Add waiting levels at the goal for the remaining time
        //int waiting_time = makespan - shortest_path_length;
        //if (waiting_time > 0) {
            // Add empty levels for waiting time
        //    for (int wait_level = shortest_path_length; wait_level < makespan; ++wait_level) {
        //        mdd->levels[wait_level] = {}; // Empty level SHOULD hold the goal tho
        //    }
            
        //    std::cout << "  Agent " << agent_id << ": shortest_path=" << shortest_path_length 
        //              << ", waiting_time=" << waiting_time << ", total_makespan=" << makespan << std::endl;
        //} else {
        //    std::cout << "  Agent " << agent_id << ": shortest_path=" << shortest_path_length 
        //              << ", no waiting time needed" << std::endl;
        //}
        
        mdds.push_back(mdd);
    }
    
    std::cout << "[LNS] Created " << mdds.size() << " MDDs with waiting time structure" << std::endl;
    return mdds;
}


// Entry point for crude LNS (skeleton). Future steps will be filled in iteratively.
int run_crude_lns(const std::string& map_path,
                  const std::string& scenario_path,
                  int num_agents,
                  int scenario_index /*0-based*/,
                  bool use_minisat /*else ProbSAT*/,
                  int seed) {
    // Step 1: Load selected scenario and map (grid, starts, goals)
    auto problem_opt = load_problem(map_path, scenario_path, num_agents, scenario_index);
    if (!problem_opt.has_value()) {
        return 1; // failed to load
    }
    const auto& problem = problem_opt.value();

    std::cout << "[LNS] Loaded map " << problem.grid.size() << "x"
              << (problem.grid.empty() ? 0 : (int)problem.grid[0].size())
              << ", agents: " << problem.starts.size() << std::endl;

    // Print full map
    std::cout << "[LNS] Map:" << std::endl;
    for (const auto& row : problem.grid) {
        for (char c : row) std::cout << c;
        std::cout << '\n';
    }

    // Print starts and goals
    std::cout << "[LNS] Agents (start -> goal):" << std::endl;
    for (size_t i = 0; i < problem.starts.size(); ++i) {
        const auto& s = problem.starts[i];
        const auto& g = problem.goals[i];
        std::cout << "  Agent " << i << ": (" << s.first << "," << s.second << ") -> ("
                  << g.first << "," << g.second << ")" << std::endl;
    }

    // Step 2: Compute base makespan (minimum feasible timesteps)
    auto [distance_matrices, base_makespan] = SATSolverManager::compute_max_timesteps(
        problem.grid, problem.starts, problem.goals);
    if (base_makespan <= 0) base_makespan = 1;

    // Outer loop: increase max timesteps if no solution is found
    const int max_timestep_increase = 10; // crude limit for now
    std::mt19937 rng(static_cast<unsigned int>(seed));
    
    for (int inc = 0; inc <= max_timestep_increase; ++inc) {
        int current_max_timesteps = base_makespan + inc;
        std::cout << "\n[LNS] === Attempt with max_timesteps=" << current_max_timesteps << " ===" << std::endl;

        // Collision tracking scoped to a single makespan attempt
        std::vector<std::tuple<int, int, std::pair<int,int>, int>> global_discovered_vertex_collisions;
        std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> global_discovered_edge_collisions;

        // Build MDDs with shortest paths + waiting time structure
        auto mdds = create_mdds_with_waiting_time(
            problem.grid, problem.starts, problem.goals, current_max_timesteps, distance_matrices);
        std::cout << "[LNS] Built MDDs with waiting time structure for " << mdds.size()
                  << " agents at makespan " << current_max_timesteps << std::endl;

        // Sample one random path per agent to get a (likely faulty) full solution
        // TO DO: sample with weighted random selection based on MDD density for overlapping nodes
            // avoiding overcrowded areas
            // each node has between one and five exit points with equal probability (so far)
            // for every possible collision on a certain node make it less likely to be chosen 
            // (maybe also reduce probablity of all prior nodes in this path?)
        // TO DO: do not sample waiting actions in random path sampling
            // every agent gets afap to the goal and if there is more time, they wait at the goal
            // Every agent that arrives early has contingent of waiting time at the goal
            // every agent with waiting time contingent can use waiting time during conflict resolution
                // if a conflict is solved by waiting time 
                    // we need to update the waiting time contingent
                    // we need to update the path that goes out of the conflict zone (every position one timestep later)
                    // during conflict resolution we can first try solving by waiting time
                    // then by expanding the conflict zone, if it is expanded, waiting time should be reset at first and only used if necessary
        // Step 1: Sample paths and store as current solution
        std::cout << "[LNS] Step 1: Sampling shortest paths and creating current solution..." << std::endl;
        int rows = (int)problem.grid.size();
        int cols = (int)problem.grid[0].size();
        int num_agents = (int)mdds.size();
        CurrentSolution current_solution(rows, cols, current_max_timesteps, num_agents);
        
        for (size_t agent_id = 0; agent_id < mdds.size(); ++agent_id) {
            const auto& mdd = mdds[agent_id];
            // Sample shortest path (agents get afap to the goal and wait there if necessary)
            auto path_positions = mdd->sample_random_path(rng);
            std::vector<std::pair<int,int>> as_pairs(path_positions.begin(), path_positions.end());
            current_solution.agent_paths[static_cast<int>(agent_id)] = std::move(as_pairs);
        }
        
        // Pad all paths to makespan and then calculate waiting times
        current_solution.pad_paths_to_makespan();
        current_solution.calculate_waiting_times(problem.goals, current_max_timesteps);
        
        // Create path map by "drawing" all agent paths
        current_solution.create_path_map();
        std::cout << "[LNS] Created current solution with " << current_solution.agent_paths.size() 
                  << " agent paths and path map of size " << rows << "x" << cols << std::endl;


building_buckets:
        // Step 2: Conflict analysis
        std::cout << "[LNS] Step 2: Analyzing conflicts..." << std::endl;
        auto [vertex_collisions, edge_collisions] = SATSolverManager::find_all_collisions(current_solution.agent_paths);
        std::cout << "[LNS] Current solution: vertex collisions=" << vertex_collisions.size()
                  << ", edge collisions=" << edge_collisions.size() << std::endl;

        // If no conflicts, we're done; else proceed to conflict zones and local solves
        if (vertex_collisions.empty() && edge_collisions.empty()) {
            std::cout << "[LNS] Collision-free sampled solution found at makespan " << current_max_timesteps << std::endl;
            std::cout << "\n[LNS] Final agent paths:" << std::endl;
            SATSolverManager::print_agent_paths(current_solution.agent_paths);
            return 0;
        }


        // - Define conflict zones/windows for each detected conflict
        //zones should have minimum size (by the offset)
        //dont ahve rectangle zones but only push boundaries as needed?? (expand along path of conflicting agents or all agents paths)
            // - For each zone, create masked map and local agent subsets
            // - Construct local MDDs and CNF for each conflict subproblem
            // - Solve each conflict subproblem with SAT/ProbSAT first if fails then CDCL; 
                // - solve in parallel (whatabout kasskades) vs serial (safe and easy)
                    //  - rank conflict zones based on how many conflicts and how many agents are involved
                // - expand window on failure
                    // - either expand window rectancle or diamond(more alligned with timesteps)
                    // - or expand window along the path of conlficting agents (minimum)
                    // - if a window touches another window, and then one of them needs to be expanded, merge them
                // - keep track of all conflicts and their prevention clauses 
                    // - use these prevention clauses to prevent collisions in the global solution paths (in case no solution found)
        // - Integrate local solutions into the global solution paths
        // - Re-check for remaining conflicts and iterate until none (solved) or limit
        // Define conflict zones (buckets) using offset

      
        const int offset = 1; // half the standard conflict zone size
        
        // Build conflict metadata from detected collisions
        std::vector<std::pair<int,int>> conflict_points;
        std::vector<ConflictMeta> conflict_meta;

        collect_conflicts(vertex_collisions, edge_collisions, conflict_points, conflict_meta);

        // Track conflicts solved/consumed across bucket resolutions in this makespan attempt
        // This can be used to rebuild or skip buckets in a future iteration step
        std::set<int> solved_conflict_indices;

        // Step 3: Build conflict buckets
        std::cout << "[LNS] Step 3: Building conflict buckets..." << std::endl;
        
        // Create spatial conflict map for efficient queries
        auto conflict_map = create_conflict_map(conflict_points, rows, cols);

      
        auto diamond_buckets = build_diamond_buckets(conflict_points, conflict_map, problem.grid, solved_conflict_indices, conflict_meta, offset);
        // If no buckets could be formed, avoid infinite rebuild loops
        if (diamond_buckets.empty()) {
            std::cout << "[LNS] No conflict buckets created; stopping this attempt to avoid looping." << std::endl;
            break; // break out of the current makespan attempt/iteration
        }
        // Validate that all conflicts were assigned to exactly one bucket
        std::vector<char> all_conflicts_used(conflict_points.size(), 0);
        for (const auto& bucket : diamond_buckets) {
            for (int conflict_idx : bucket.indices) {
                if (all_conflicts_used[conflict_idx]) {
                    std::cerr << "[ERROR] Conflict " << conflict_idx
                              << " was assigned to multiple diamond buckets. "
                              << "This violates the closed neighborhood design." << std::endl;
                    return 1;
                }
                all_conflicts_used[conflict_idx] = 1;
            }
        }
        
        // Check for unused conflicts
        int unused_conflicts = 0;
        for (size_t i = 0; i < all_conflicts_used.size(); ++i) {
            if (!all_conflicts_used[i]) unused_conflicts++;
        }
        
        if (unused_conflicts > 0) {
            std::cerr << "[WARNING] " << unused_conflicts
                      << " conflicts were not assigned to any diamond bucket." << std::endl;
        }
        
        std::cout << "[LNS] Formed " << diamond_buckets.size() << " diamond-shaped conflict bucket(s)." << std::endl;
        for (size_t bi = 0; bi < diamond_buckets.size(); ++bi) {
            const auto& b = diamond_buckets[bi];
            std::cout << "  Diamond Bucket " << bi << ": positions=" << b.positions.size() 
                      << ", conflicts=" << b.indices.size() << std::endl;
        }
    
select_bucket:
        
        // Step 4: Decide which bucket to tackle
        std::cout << "[LNS] Step 4: Selecting most relevant bucket..." << std::endl;
        
        // Filter out buckets fully eliminated by already solved conflicts
        std::vector<int> remaining_bucket_indices;
        remaining_bucket_indices.reserve(diamond_buckets.size());
        for (size_t bi = 0; bi < diamond_buckets.size(); ++bi) {
            const auto& bucket = diamond_buckets[bi];
            bool has_unsolved = false;
            for (int idx : bucket.indices) {
                if (solved_conflict_indices.find(idx) == solved_conflict_indices.end()) {
                    has_unsolved = true;
                    break;
                }
            }
            if (has_unsolved) remaining_bucket_indices.push_back((int)bi);
        }
        if (remaining_bucket_indices.empty()) {
            std::cout << "[LNS] No remaining buckets after integration; proceeding to post-processing." << std::endl;
            // Global evaluation: check if current solution has any conflicts left
            std::cout << "[LNS] NO remaining buckets, Global evaluation of current solution..." << std::endl;
            
            auto [glob_v_coll, glob_e_coll] = SATSolverManager::find_all_collisions(current_solution.agent_paths);
            if (glob_v_coll.empty() && glob_e_coll.empty()) {
                std::cout << "[LNS] No conflicts remain in the global solution. SOLVED at makespan "
                            << current_max_timesteps << std::endl;
                std::cout << "\n[LNS] Final agent paths:" << std::endl;
                SATSolverManager::print_agent_paths(current_solution.agent_paths);
                return 0;
            }
            std::cout << "[LNS] Global conflicts remain: vertex=" << glob_v_coll.size()
                        << ", edge=" << glob_e_coll.size() << ". Rebuilding buckets..." << std::endl;

            // Rebuild conflict_points and conflict_meta from current global conflicts
            collect_conflicts(glob_v_coll, glob_e_coll, conflict_points, conflict_meta);

            // Reset solved set since indices refer to freshly rebuilt arrays
            solved_conflict_indices.clear();

            // Rebuild buckets from scratch for the remaining conflicts
            goto building_buckets;
        }

        // Find the most relevant diamond bucket among remaining
        int best_bucket_idx = -1;
        int best_conflict_count = -1;
        int best_shape_size = -1;
        for (int bi : remaining_bucket_indices) {
            const auto& bucket = diamond_buckets[bi];
            int conflict_count = 0;
            for (int idx : bucket.indices) {
                if (solved_conflict_indices.find(idx) == solved_conflict_indices.end()) {
                    conflict_count++;
                }
            }
            int shape_size = (int)bucket.positions.size();
            bool is_better = false;
            if (conflict_count > best_conflict_count) {
                is_better = true;
            } else if (conflict_count == best_conflict_count && shape_size > best_shape_size) {
                is_better = true;
            }
            if (is_better) {
                best_bucket_idx = bi;
                best_conflict_count = conflict_count;
                best_shape_size = shape_size;
            }
        }
        
        if (best_bucket_idx == -1) {
            std::cerr << "[ERROR] No diamond buckets found to process." << std::endl;
            return 1;
        }
        
        const auto& best_bucket = diamond_buckets[best_bucket_idx];
        std::cout << "[LNS] Selected most relevant diamond bucket " << best_bucket_idx 
                << " with " << best_conflict_count << " conflicts and " 
                << best_shape_size << " positions." << std::endl;

        //now we can solve that bucket with 
        // Step 5: Create map crop for the selected bucket
        std::cout << "[LNS] Step 5: Creating map crop for selected bucket..." << std::endl;
        auto masked_map = mask_map_outside_shape(problem.grid, best_bucket.positions);
        
        // Step 6: Compute time window [earliest_conflict-offset, latest_conflict+offset]
        std::cout << "[LNS] Step 6: Computing conflict time window and extracting local paths..." << std::endl;
        std::set<std::pair<int,int>> zone_positions_set(best_bucket.positions.begin(), best_bucket.positions.end());
        int earliest_conflict_t = INT_MAX;
        int latest_conflict_t = -1;
        for (int idx : best_bucket.indices) {
            if (idx < 0 || idx >= (int)conflict_meta.size()) continue;
            int t = conflict_meta[idx].timestep;
            if (t < earliest_conflict_t) earliest_conflict_t = t;
            if (t > latest_conflict_t) latest_conflict_t = t;
        }
        if (earliest_conflict_t == INT_MAX) {
            earliest_conflict_t = 0;
            latest_conflict_t = current_solution.max_timestep;
        }
        int start_t = std::max(0, earliest_conflict_t - offset);
        int end_t = std::min(current_solution.max_timestep, latest_conflict_t + offset);
        std::cout << "[LNS] Conflict time window: [" << start_t << ", " << end_t << "]" << std::endl;
        
        // Agents present in the zone within the time window
        std::set<int> agents_in_window;
        for (const auto& rc : zone_positions_set) {
            int r = rc.first, c = rc.second;
            for (int t = start_t; t <= end_t; ++t) {
                auto here = current_solution.get_agents_at_position_time(r, c, t);
                agents_in_window.insert(here.begin(), here.end());
            }
        }
        std::cout << "[LNS] Agents in zone within window: " << agents_in_window.size() << std::endl;
        
        // Step 7: For each agent, extract its subpath inside (zone × [start_t,end_t])
        std::unordered_map<int, std::vector<std::pair<int,int>>> local_zone_paths;
        std::unordered_map<int, std::pair<int,int>> local_entry_exit_time; // agent -> {entry_t, exit_t}
        for (int agent_id : agents_in_window) {
            const auto& path = current_solution.agent_paths.at(agent_id);
            std::vector<std::pair<int,int>> segment;
            int entry_t = -1, exit_t = -1;
            for (int t = start_t; t <= end_t && t < (int)path.size(); ++t) {
                const auto& pos = path[t];
                if (zone_positions_set.count(pos)) {
                    if (entry_t == -1) entry_t = t;
                    exit_t = t;
                    segment.push_back(pos);
                } else if (exit_t != -1) {
                    break; // left the zone after entering
                }
            }
            if (!segment.empty()) {
                local_zone_paths[agent_id] = std::move(segment);
                local_entry_exit_time[agent_id] = {entry_t, exit_t};
                std::cout << "  Agent " << agent_id << " local segment t=[" << entry_t << "," << exit_t
                        << "] len=" << (exit_t - entry_t + 1) << std::endl;
            }
        }

        // Step 8: Create aligned MDDs for each agent in the zone
        std::cout << "[LNS] Step 8: Creating aligned MDDs for local problem..." << std::endl;
        
        // All MDDs must have the same length (aligned to the zone time window)
        int zone_mdd_length = end_t - start_t + 1;
        std::cout << "[LNS] Zone MDD length: " << zone_mdd_length << " (from t=" << start_t << " to t=" << end_t << ")" << std::endl;
        
        std::unordered_map<int, std::shared_ptr<MDD>> local_mdds;
        
        for (const auto& [agent_id, entry_exit_time] : local_entry_exit_time) {
            int entry_t = entry_exit_time.first;
            int exit_t = entry_exit_time.second;
            int agent_path_length = exit_t - entry_t + 1;
            
            // Get agent's zone entry and exit points from their local path
            const auto& local_path = local_zone_paths[agent_id];
            auto zone_start_pos = local_path.front();  // Entry point into zone
            auto zone_goal_pos = local_path.back();    // Exit point from zone
            
            // Create MDD with the agent's specific path length (not the full zone window)
            MDDConstructor constructor(masked_map, zone_start_pos, zone_goal_pos, agent_path_length - 1);
            auto agent_mdd = constructor.construct_mdd();
            

            // Align MDD to the zone time window
            align_mdd_to_time_window(agent_mdd, entry_t, exit_t, start_t, end_t);
            
            local_mdds[agent_id] = agent_mdd;
            
            int relative_entry = entry_t - start_t;
            int relative_exit = exit_t - start_t;
            std::cout << "  Agent " << agent_id << " MDD: start=(" << zone_start_pos.first << "," << zone_start_pos.second 
                    << ") goal=(" << zone_goal_pos.first << "," << zone_goal_pos.second 
                    << ") path_length=" << agent_path_length 
                    << " active levels=[" << relative_entry << "," << relative_exit << "] of " << zone_mdd_length << std::endl;
        }
        
        std::cout << "[LNS] Created " << local_mdds.size() << " local MDDs" << std::endl;
        
        // Step 9: Create CNF for the local problem (lazy encoding)
        std::cout << "[LNS] Step 9: Creating CNF for local problem..." << std::endl;
        
        // Step 9a: Create lazy CNF from MDDs (no conflict clauses yet)
        CNFConstructor cnf_constructor(local_mdds, true); // true = lazy encoding
        CNF local_cnf = cnf_constructor.construct_cnf();
        
        std::cout << "[LNS] Created lazy CNF with " << local_cnf.get_clauses().size() 
                << " clauses and " << (cnf_constructor.get_next_variable_id() - 1) << " variables" << std::endl;
        
        // Step 9b: Extract vertex collision clauses for lazy solving
        std::cout << "[LNS] Step 9b: Extracting vertex collision clauses..." << std::endl;
        int number_of_vertex_collisions = 0;
        std::vector<std::tuple<int, int, std::pair<int,int>, int>> initial_vertex_collisions;
        for (int idx : best_bucket.indices) {
            if (idx < 0 || idx >= (int)conflict_meta.size()) continue;
            
            const auto& meta = conflict_meta[idx];
            if (!meta.is_edge) {
                // Track this collision for lazy solving
                initial_vertex_collisions.emplace_back(meta.agent1, meta.agent2, meta.pos1, meta.timestep);
                number_of_vertex_collisions++;
            }
        }
        std::cout << "[LNS] Extracted " << number_of_vertex_collisions << " vertex collision clauses" << std::endl;
        
        // Step 9c: Extract edge collision clauses for lazy solving
        std::cout << "[LNS] Step 9c: Extracting edge collision clauses..." << std::endl;
        std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> initial_edge_collisions;
        //all edge collisions are twice in the metadata, so we need to remove duplicates
        std::set<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> initial_edge_collisions_set;
        for (int idx : best_bucket.indices) {
            if (idx < 0 || idx >= (int)conflict_meta.size()) continue;
            
            const auto& meta = conflict_meta[idx];
            if (meta.is_edge) {
                //check if we alread track this collision
                if (initial_edge_collisions_set.count(std::make_tuple(
                        meta.agent1, meta.agent2, meta.pos1, meta.pos2, meta.timestep)) > 0) {
                    continue;
                }
                initial_edge_collisions_set.insert(std::make_tuple(
                    meta.agent1, meta.agent2, meta.pos1, meta.pos2, meta.timestep));
                // Track this collision for lazy solving
                initial_edge_collisions.emplace_back(meta.agent1, meta.agent2, meta.pos1, meta.pos2, meta.timestep);
            }
        }
        std::cout << "[LNS] Extracted " << initial_edge_collisions.size() << " edge collision clauses" << std::endl;
        
        std::cout << "[LNS] Final local CNF: " << local_cnf.get_clauses().size() 
                << " total clauses" << std::endl;
        
        // Step 10: Lazy solving of conflict zone
        std::cout << "[LNS] Step 10: Starting lazy solving of conflict zone..." << std::endl;
        
        auto lazy_result = lazy_solve_conflict_zone(
            local_cnf, cnf_constructor, local_entry_exit_time, start_t, end_t,
            1000000, // max_iterations
            &initial_vertex_collisions, // Pass initial discovered collisions
            &initial_edge_collisions);

        if (lazy_result.solution_found) {
            // Update global solution with local paths using helper function
            current_solution.update_with_local_paths(lazy_result.local_paths, local_entry_exit_time);
            
            // Update global collision tracking with all collisions discovered in this bucket
            global_discovered_vertex_collisions.insert(global_discovered_vertex_collisions.end(),
                                                    lazy_result.discovered_vertex_collisions.begin(),
                                                    lazy_result.discovered_vertex_collisions.end());
            global_discovered_edge_collisions.insert(global_discovered_edge_collisions.end(),
                                                lazy_result.discovered_edge_collisions.begin(),
                                                lazy_result.discovered_edge_collisions.end());
            
            std::cout << "[LNS] Updated global collision tracking: " << global_discovered_vertex_collisions.size() 
                    << " vertex collisions, " << global_discovered_edge_collisions.size() << " edge collisions" << std::endl;

            // Case 1: Solved without waiting-time and without expansion
            // Mark all conflicts in this bucket as solved/consumed
            for (int idx : best_bucket.indices) {
                if (idx >= 0 && idx < (int)conflict_meta.size()) {
                    solved_conflict_indices.insert(idx);
                }
            }
            std::cout << "[LNS] Bucket solved without expansion. Marked " << best_bucket.indices.size()
                    << " conflicts as solved. Selecting next bucket..." << std::endl;
            goto select_bucket;
        }

        if (!lazy_result.solution_found) {
            std::cout << "[LNS] Local problem appears to be unsatisfiable in current zone" << std::endl;
            
            // Progressive expansion strategy: try different expansion factors
            bool expansion_solution_found = false;
            int expansion_factor = 0;
            const int max_expansion_factor = 100; // Maximum expansion attempts
            
            // Backup waiting times before starting expansion strategy
            auto waiting_time_backup = current_solution.backup_waiting_times();
            
            // Persistent collision tracking for this bucket across all expansion attempts
            // Start with collisions discovered from the initial lazy solve attempt
            std::vector<std::tuple<int, int, std::pair<int,int>, int>> bucket_discovered_vertex_collisions = lazy_result.discovered_vertex_collisions;
            std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> bucket_discovered_edge_collisions = lazy_result.discovered_edge_collisions;
            
            while (!expansion_solution_found && expansion_factor <= max_expansion_factor) {
                std::cout << "[LNS] Attempting resolution with expansion factor " << expansion_factor << "..." << std::endl;
                
                if (expansion_factor == 0) {
                    // First attempt: use waiting time strategy with original bucket
                    std::cout << "[LNS] Trying waiting time strategy with original bucket..." << std::endl;
                    
                    auto waiting_time_result = lazy_solve_with_waiting_time(
                        current_solution, masked_map, zone_positions_set, local_mdds, local_zone_paths, local_entry_exit_time,
                        bucket_discovered_vertex_collisions, bucket_discovered_edge_collisions, conflict_meta, best_bucket.indices, start_t, end_t);
                    
                    if (waiting_time_result.solution_found) {
                        expansion_solution_found = true;
                        
                        // update solved conflict indices
                        for (int idx_cons : best_bucket.indices) {
                            if (idx_cons >= 0 && idx_cons < (int)conflict_meta.size()) {
                                solved_conflict_indices.insert(idx_cons);
                            }
                        }
                        // Update global collision tracking with all collisions discovered in this bucket
                        global_discovered_vertex_collisions.insert(global_discovered_vertex_collisions.end(),
                                                                waiting_time_result.discovered_vertex_collisions.begin(),
                                                                waiting_time_result.discovered_vertex_collisions.end());
                        global_discovered_edge_collisions.insert(global_discovered_edge_collisions.end(),
                                                            waiting_time_result.discovered_edge_collisions.begin(),
                                                            waiting_time_result.discovered_edge_collisions.end());
                        
                        std::cout << "[LNS] Updated global collision tracking: " << global_discovered_vertex_collisions.size() 
                                << " vertex collisions, " << global_discovered_edge_collisions.size() << " edge collisions" << std::endl;

                        //rebuild buckets because we delayed agents in the zone with waiting time, changing their whole path globally
                        goto building_buckets;
                    } else {
                        std::cout << "[LNS] Waiting time strategy failed, checking bucket size..." << std::endl;
                        
                        // Check if the original bucket already covers the entire map
                        int total_map_cells = rows * cols;
                        double original_bucket_coverage = (double)best_bucket.positions.size() / total_map_cells;
                        std::cout << "[LNS] Original bucket coverage: " << (original_bucket_coverage * 100.0) << "% of map (" 
                                << best_bucket.positions.size() << "/" << total_map_cells << " cells)" << std::endl;
                        
                        if (original_bucket_coverage >= 1.0) {
                            std::cout << "[LNS] The original bucket covers 100% of the map - this is already a global problem!" << std::endl;
                            std::cout << "[LNS] Since waiting time strategy failed on the global problem, we need to increase makespan." << std::endl;
                            std::cout << "[LNS] Breaking out of current makespan attempt..." << std::endl;
                            
                            // Break out of the current makespan attempt directly
                            goto increase_makespan; // Jump to end of makespan attempt
                        } else {
                            std::cout << "[LNS] Original bucket coverage is " << (original_bucket_coverage * 100.0) 
                                    << "%, can still expand. Will try expansion factor " << (expansion_factor + 1) << std::endl;
                        }
                        
                        // Update bucket collision tracking (waiting_time_result already contains accumulated collisions)
                        bucket_discovered_vertex_collisions = waiting_time_result.discovered_vertex_collisions;
                        bucket_discovered_edge_collisions = waiting_time_result.discovered_edge_collisions;
                    }
                } else {
                    // Expansion attempts: increase bucket offset and try again
                    std::cout << "[LNS] Expanding bucket with expansion factor " << expansion_factor << "..." << std::endl;
                    
                    // Step 1: Restore original waiting times
                    std::cout << "[LNS] Restoring original waiting times..." << std::endl;
                    current_solution.restore_waiting_times(waiting_time_backup);
                    
                    // Step 2: Create expanded bucket with increased offset
                    int expanded_offset = offset + expansion_factor;
                    std::cout << "[LNS] Creating expanded bucket with offset " << expanded_offset << " (original: " << offset << ")" << std::endl;
                    
                    // Recreate the bucket conflicts from the original bucket indices
                    std::vector<std::pair<int,int>> expanded_bucket_conflicts;
                    std::vector<int> expanded_conflict_indices;
                    for (int idx : best_bucket.indices) {
                        if (idx >= 0 && idx < (int)conflict_points.size()) {
                            expanded_bucket_conflicts.push_back(conflict_points[idx]);
                            expanded_conflict_indices.push_back(idx);
                        }
                    }
                    
                    // Create expanded diamond shape with increased offset
                    auto expanded_diamond_positions = create_shape_from_conflicts(expanded_bucket_conflicts, expanded_offset);
                    std::cout << "[LNS] Expanded bucket contains " << expanded_diamond_positions.size() << " positions (original: " << best_bucket.positions.size() << ")" << std::endl;
                    
                    // Find and merge newly touched conflicts in the expanded zone
                    // Initialize previous_bucket_shape for iterative conflict discovery
                    // Start with the original bucket shape so we catch conflicts from the initial expansion
                    std::set<std::pair<int,int>> previous_bucket_shape(best_bucket.positions.begin(), best_bucket.positions.end());
                    auto new_positions_in_expansion = find_new_positions(expanded_diamond_positions, previous_bucket_shape);
                    
                
                    
                    std::cout << "[LNS] Checking " << new_positions_in_expansion.size() << " new positions for conflicts (efficient approach)" << std::endl;
                    
                    std::vector<int> newly_touched_conflicts;
                    
                    
                    // Check only the new positions for conflicts (efficient!)
                    for (const auto& pos : new_positions_in_expansion) {
                        auto [r, c] = pos;
                        if (conflict_map[r][c].empty()) continue;
                        for (int conflict_idx : conflict_map[r][c]) {
                            //check if the conflict is already solved
                            if (solved_conflict_indices.find(conflict_idx) != solved_conflict_indices.end()) continue;
                            // Check if this conflict was not part of the original bucket
                            bool was_in_original_bucket = false;
                            for (int original_idx : best_bucket.indices) {
                                if (original_idx == conflict_idx) {
                                    was_in_original_bucket = true;
                                    break;
                                }
                            }
                            
                            if (!was_in_original_bucket) {
                                newly_touched_conflicts.push_back(conflict_idx);
                                std::cout << "  Found newly touched conflict " << conflict_idx << " at position (" << r << "," << c << ")" << std::endl;
                            } else {
                                std::cout << "[LNS] ERROR: Conflict in new zone " << conflict_idx << " was already in the original bucket/previous zone" << std::endl;
                            }
                        }
                    }
                    
                    if (!newly_touched_conflicts.empty()) { //we have new conflicts to add to the expanded bucket, need to expand the bucket further
                        std::cout << "[LNS] Merging " << newly_touched_conflicts.size() << " newly touched conflicts into expanded bucket" << std::endl;
                        
                        //set previous bucket shape to the expanded diamond positions (as we checked for conflicts in the expanded diamond shape)
                        previous_bucket_shape = std::set<std::pair<int,int>>(expanded_diamond_positions.begin(), expanded_diamond_positions.end());

                        // Add newly touched conflicts to the expanded bucket conflicts
                        for (int new_conflict_idx : newly_touched_conflicts) {
                            if (new_conflict_idx >= 0 && new_conflict_idx < (int)conflict_points.size()) {
                                expanded_bucket_conflicts.push_back(conflict_points[new_conflict_idx]);
                                expanded_conflict_indices.push_back(new_conflict_idx);
                            }
                        }
                        
                        //create the new expanded diamond shape with the merged conflicts
                        expanded_diamond_positions = create_shape_from_conflicts(expanded_bucket_conflicts, expanded_offset);
                        std::cout << "[LNS] After merging new conflicts, expanded bucket contains " << expanded_diamond_positions.size() << " positions" << std::endl;
                        
                        // Iteratively check for more conflicts in the newly gained area
                        // This is similar to the original bucket creation process
                        bool found_more_conflicts = true;
                        int iteration = 0;
                        
                        
                        while (found_more_conflicts) { //keep expanding the bucket until we find no more conflicts that are in the expanded area
                            iteration++;
                            found_more_conflicts = false;
                            
                            // Create current shape from all conflicts in expanded bucket
                            auto current_expanded_shape = create_shape_from_conflicts(expanded_bucket_conflicts, expanded_offset);
                            
                            // Find new positions that weren't in the previous shape
                            auto new_positions_from_merged = find_new_positions(current_expanded_shape, previous_bucket_shape);
                            
                            std::cout << "[LNS] Iteration " << iteration << ": Checking " << new_positions_from_merged.size() 
                                    << " new positions from merged conflicts" << std::endl;
                            
                            // Check only the new positions for conflicts
                            for (const auto& pos : new_positions_from_merged) {
                                auto [r, c] = pos;
                                if (conflict_map[r][c].empty()) continue;
                                for (int conflict_idx : conflict_map[r][c]) {
                                    //check if the conflict is already solved
                                    if (solved_conflict_indices.find(conflict_idx) != solved_conflict_indices.end()) continue;
                                    // Check if this conflict was not already in our expanded bucket
                                    bool already_in_expanded_bucket = false;
                                    for (int existing_idx : expanded_conflict_indices) {
                                        if (existing_idx == conflict_idx) {
                                            already_in_expanded_bucket = true;
                                            break;
                                        }
                                    }
                                    
                                    if (!already_in_expanded_bucket) {
                                        // Found another new conflict in the expanded area
                                        expanded_bucket_conflicts.push_back(conflict_points[conflict_idx]);
                                        expanded_conflict_indices.push_back(conflict_idx);
                                        newly_touched_conflicts.push_back(conflict_idx); //log for adding clauses to cnf later
                                        found_more_conflicts = true;
                                        std::cout << "  Found additional conflict " << conflict_idx << " at position (" << r << "," << c << ")" << std::endl;
                                    }
                                }
                            }
                            
                            // Update the previous bucket shape for next iteration
                            previous_bucket_shape = std::set<std::pair<int,int>>(current_expanded_shape.begin(), current_expanded_shape.end());
                            
                            if (found_more_conflicts) {
                                std::cout << "[LNS] Found " << expanded_bucket_conflicts.size() - (best_bucket.indices.size() + newly_touched_conflicts.size()) 
                                        << " additional conflicts in iteration " << iteration << std::endl;
                            }
                        }
                        
                        
                        // Final expanded diamond with all discovered conflicts
                        expanded_diamond_positions = create_shape_from_conflicts(expanded_bucket_conflicts, expanded_offset);
                        std::cout << "[LNS] Final expanded bucket contains " << expanded_diamond_positions.size() 
                                << " positions with " << expanded_bucket_conflicts.size() << " conflicts" << std::endl;
                    }
                    
                    // Step 3: create new zone with expanded bucket
                    std::cout << "[LNS] creating zone with expanded bucket..." << std::endl;
                    
                    // Update zone positions set with expanded diamond
                    std::set<std::pair<int,int>> expanded_zone_positions_set(expanded_diamond_positions.begin(), expanded_diamond_positions.end());
                    
                    // Recreate local zone paths for the expanded zone
                    std::unordered_map<int, std::vector<std::pair<int,int>>> expanded_local_zone_paths;
                    std::unordered_map<int, std::pair<int,int>> expanded_local_entry_exit_time;
                    std::unordered_map<int, std::shared_ptr<MDD>> expanded_local_mdds;
                    
                    // Find all agents that pass through the expanded zone
                    std::set<int> expanded_agents_in_zone;
                    for (const auto& [agent_id, path] : current_solution.agent_paths) {
                        for (int t = 0; t < (int)path.size(); ++t) {
                            if (expanded_zone_positions_set.count(path[t])) {
                                expanded_agents_in_zone.insert(agent_id);
                                break;
                            }
                        }
                    }
                    
                    std::cout << "[LNS] Found " << expanded_agents_in_zone.size() << " agents in expanded zone (original: " << local_zone_paths.size() << ")" << std::endl;
                    
                    // Extract local paths for agents in expanded zone
                    for (int agent_id : expanded_agents_in_zone) {
                        const auto& path = current_solution.agent_paths.at(agent_id);
                        std::vector<std::pair<int,int>> segment;
                        int entry_t = -1, exit_t = -1;
                        
                        // Find entry and exit times for this agent in the expanded zone
                        for (int t = 0; t < (int)path.size(); ++t) {
                            if (expanded_zone_positions_set.count(path[t])) {
                                if (entry_t == -1) entry_t = t;
                                exit_t = t;
                                segment.push_back(path[t]);
                            }
                        }
                        
                        if (!segment.empty()) {
                            expanded_local_zone_paths[agent_id] = std::move(segment);
                            expanded_local_entry_exit_time[agent_id] = {entry_t, exit_t};
                            std::cout << "[LNS] ERROR:  Agent " << agent_id << " local segment t=[" << entry_t << "," << exit_t
                                    << "] len=" << (exit_t - entry_t + 1) << std::endl;
                        }
                    }
                    
                    // Create MDDs for agents in expanded zone
                    for (const auto& [agent_id, local_path] : expanded_local_zone_paths) {
                        auto entry_exit = expanded_local_entry_exit_time[agent_id];
                        int entry_t = entry_exit.first;
                        int exit_t = entry_exit.second;
                        
                        auto zone_start_pos = local_path.front();
                        auto zone_goal_pos = local_path.back();
                        int agent_path_length = exit_t - entry_t + 1;
                        
                        MDDConstructor constructor(masked_map, zone_start_pos, zone_goal_pos, agent_path_length - 1);
                        auto agent_mdd = constructor.construct_mdd();
                        
                        // Align MDD to the time window
                        align_mdd_to_time_window(agent_mdd, entry_t, exit_t, start_t, end_t);
                        
                        expanded_local_mdds[agent_id] = agent_mdd;
                    }
                    
                    // Step 4: Try waiting time strategy with expanded zone
                    std::cout << "[LNS] Trying waiting time strategy with expanded zone..." << std::endl;
                    
                    // alread added newly touched conflicts to expanded conflict indices
                    //expanded_conflict_indices.insert(expanded_conflict_indices.end(), newly_touched_conflicts.begin(), newly_touched_conflicts.end());
                    
                    // Extract collision clauses for newly found conflicts
                    std::vector<std::tuple<int, int, std::pair<int,int>, int>> new_vertex_collisions;
                    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> new_edge_collisions;
                    
                    for (int new_conflict_idx : newly_touched_conflicts) {
                        if (new_conflict_idx >= 0 && new_conflict_idx < (int)conflict_meta.size()) {
                            const auto& meta = conflict_meta[new_conflict_idx];
                            if (!meta.is_edge) {
                                // Track this vertex collision for lazy solving
                                new_vertex_collisions.emplace_back(meta.agent1, meta.agent2, meta.pos1, meta.timestep);
                            } else {
                                // Track this edge collision for lazy solving
                                new_edge_collisions.emplace_back(meta.agent1, meta.agent2, meta.pos1, meta.pos2, meta.timestep);
                            }
                        }
                    }
            
                    // Add new collision clauses to bucket collision tracking
                    bucket_discovered_vertex_collisions.insert(bucket_discovered_vertex_collisions.end(), 
                                                            new_vertex_collisions.begin(), new_vertex_collisions.end());
                    bucket_discovered_edge_collisions.insert(bucket_discovered_edge_collisions.end(),
                                                        new_edge_collisions.begin(), new_edge_collisions.end());
                    
                    std::cout << "[LNS] Added " << new_vertex_collisions.size() << " new vertex collisions and " 
                            << new_edge_collisions.size() << " new edge collisions to bucket tracking" << std::endl;
                    std::cout << "[LNS] Using " << expanded_conflict_indices.size() << " conflict indices (original: " << best_bucket.indices.size() 
                            << ", newly touched: " << newly_touched_conflicts.size() << ")" << std::endl;
                    
                    //if the expanded zone is the whole map, we provided all global discovered collisions
                    if (expanded_zone_positions_set.size() == rows * cols) {
                        std::cout << "[LNS] Expanded zone is the whole map, providing all global discovered collisions" << std::endl;
                        bucket_discovered_vertex_collisions.insert(
                            bucket_discovered_vertex_collisions.end(),
                            global_discovered_vertex_collisions.begin(),
                            global_discovered_vertex_collisions.end());
                        bucket_discovered_edge_collisions.insert(
                            bucket_discovered_edge_collisions.end(),
                            global_discovered_edge_collisions.begin(),
                            global_discovered_edge_collisions.end());
                    }

                    auto expanded_waiting_time_result = lazy_solve_with_waiting_time(
                        current_solution, masked_map, expanded_zone_positions_set, expanded_local_mdds, expanded_local_zone_paths, expanded_local_entry_exit_time,
                        bucket_discovered_vertex_collisions, bucket_discovered_edge_collisions, conflict_meta, expanded_conflict_indices, start_t, end_t);
                    
                    if (expanded_waiting_time_result.solution_found) {
                        expansion_solution_found = true;
                        std::cout << "[LNS] Successfully resolved conflicts using expanded bucket with factor " << expansion_factor << "!" << std::endl;
                        //soleved with expansion
                        // Update global collision tracking with all collisions discovered in this bucket
                        global_discovered_vertex_collisions.insert(global_discovered_vertex_collisions.end(),
                                                                expanded_waiting_time_result.discovered_vertex_collisions.begin(),
                                                                expanded_waiting_time_result.discovered_vertex_collisions.end());
                        global_discovered_edge_collisions.insert(global_discovered_edge_collisions.end(),
                                                            expanded_waiting_time_result.discovered_edge_collisions.begin(),
                                                            expanded_waiting_time_result.discovered_edge_collisions.end());
                        
                        std::cout << "[LNS] Updated global collision tracking: " << global_discovered_vertex_collisions.size() 
                                << " vertex collisions, " << global_discovered_edge_collisions.size() << " edge collisions" << std::endl;

                        // Case 2: Solved after expansion
                        // Determine if this expanded bucket absorbed other conflicts/buckets
                        // expanded_conflict_indices holds original + newly touched conflicts
                        size_t original_count = best_bucket.indices.size();
                        size_t expanded_count = expanded_conflict_indices.size();
                        bool absorbed_other_buckets = expanded_count > original_count;

                        // Mark all expanded conflicts as solved/consumed
                        for (int idx_cons : expanded_conflict_indices) {
                            if (idx_cons >= 0 && idx_cons < (int)conflict_meta.size()) {
                                solved_conflict_indices.insert(idx_cons);
                            }
                        }

                        if (absorbed_other_buckets) {
                            //we have absorbed other buckets, so we need to rebuild the buckets
                            std::cout << "[LNS] Expanded bucket absorbed additional conflicts (" << (expanded_count - original_count)
                                    << "). Global solution updated. Rebuilding remaining buckets view and selecting next..." << std::endl;
                            goto building_buckets;
                        } else {
                            //we did not absorb other buckets, so we can continue with the next bucket
                            std::cout << "[LNS] Expanded bucket did not absorb other buckets. Selecting next bucket..." << std::endl;
                            goto select_bucket;
                        }
                       


                    } else {
                        std::cout << "[LNS] Expanded bucket with factor " << expansion_factor << " did not resolve conflicts, checking zone size..." << std::endl;
                        
                        // Check if the zone we just tried covers the entire map
                        int total_map_cells = rows * cols;
                        double zone_coverage_ratio = (double)expanded_diamond_positions.size() / total_map_cells;
                        std::cout << "[LNS] Zone coverage: " << (zone_coverage_ratio * 100.0) << "% of map (" 
                                  << expanded_diamond_positions.size() << "/" << total_map_cells << " cells)" << std::endl;
                        
                        if (zone_coverage_ratio >= 1.0) {
                            std::cout << "[LNS] The zone covers 100% of the map - there's nowhere else to expand to!" << std::endl;
                            std::cout << "[LNS] Since we failed to solve the global problem, we need to increase makespan." << std::endl;
                            std::cout << "[LNS] Breaking out of current makespan attempt..." << std::endl;
                            
                            // Break out of the current makespan attempt directly
                            goto increase_makespan; // Jump to end of makespan attempt
                        } else {
                            std::cout << "[LNS] Zone coverage is " << (zone_coverage_ratio * 100.0) 
                                      << "%, can still expand. Will try factor " << (expansion_factor + 1) << std::endl;
                        }
                        
                        // Update bucket collision tracking for next expansion attempt
                        bucket_discovered_vertex_collisions = expanded_waiting_time_result.discovered_vertex_collisions;
                        bucket_discovered_edge_collisions = expanded_waiting_time_result.discovered_edge_collisions;
                    }
                }
                
                expansion_factor++;
            }
            
            // If we reach here, all expansion strategies were exhausted without finding a solution
            // and without hitting 100% coverage (otherwise we would have used goto)
            std::cout << "[LNS] All expansion strategies exhausted without hitting 100% coverage." << std::endl;
            std::cout << "[LNS] This suggests the problem is too constrained at current makespan." << std::endl;
            std::cout << "[LNS] Breaking out of current makespan attempt to try with higher makespan..." << std::endl;
            std::cout << "[LNS] THIS SHOULD NOT HAPPEN, SOMETHING IS WRONG WITH THE EXPANSION STRATEGY" << std::endl;
            break; // This breaks out of the current makespan attempt
            
        } else {
            std::cout << "[LNS] Successfully resolved conflicts in selected bucket without need for waiting time strategy or expansion!" << std::endl;
            
            // TODO: Continue LNS iteration
            //if zone did not expand, we can continue with the next most relevant bucket
            // if the zone expanded we either need to redo buckets or we can check if the expanded zone touched/consumed another bucket
            // 1. Re-check for remaining conflicts in the updated global solution
            // 2. If conflicts remain, select next most relevant bucket
            // 3. If no conflicts, we have a complete solution
            
            std::cout << "[LNS] Selecting next bucket after successful local solve..." << std::endl;
            goto select_bucket;
        }



        // TODO: Use SATSolverManager to solve the local CNF
        // - Replace the conflicting paths with new solutions
        // - Update the current_solution with the new paths
        // Next steps: prepare local problems using these buckets
        // 
        
    increase_makespan:
        // Label for goto when we detect 100% coverage and need to increase makespan
        std::cout << "[LNS] Moving to next makespan attempt..." << std::endl;
    }

    return 0;
}
