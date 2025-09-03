#include "../SATSolverManager.h"
#include <vector>
#include <string>
#include <iostream>
#include <optional>
#include <random>
#include <set>

struct LNSProblem {
    std::vector<std::vector<char>> grid;
    std::vector<std::pair<int,int>> starts;
    std::vector<std::pair<int,int>> goals;
};

// Helper: create a masked map keeping only [min_row..max_row] x [min_col..max_col] walkable
static std::vector<std::vector<char>> mask_map_outside_bounds(
    const std::vector<std::vector<char>>& map,
    int min_row, int min_col, int max_row, int max_col) {
    if (map.empty() || map[0].empty()) return {};
    int rows = (int)map.size();
    int cols = (int)map[0].size();
    min_row = std::max(0, std::min(min_row, rows - 1));
    min_col = std::max(0, std::min(min_col, cols - 1));
    max_row = std::max(0, std::min(max_row, rows - 1));
    max_col = std::max(0, std::min(max_col, cols - 1));
    if (min_row > max_row || min_col > max_col) return std::vector<std::vector<char>>(rows, std::vector<char>(cols, '@'));

    std::vector<std::vector<char>> masked(rows, std::vector<char>(cols, '@'));
    for (int r = min_row; r <= max_row; ++r) {
        for (int c = min_col; c <= max_col; ++c) masked[r][c] = map[r][c];
    }
    return masked;
}

// Loads the map and the requested scenario entry, extracting starts/goals for num_agents
static std::optional<LNSProblem> load_problem(const std::string& map_path,
                                              const std::string& scenario_path,
                                              int num_agents,
                                              int scenario_index /*0-based*/) {
    LNSProblem problem;

    // Load map
    problem.grid = SATSolverManager::load_map(map_path);
    if (problem.grid.empty()) {
        std::cerr << "[LNS] Failed to load map from: " << map_path << std::endl;
        return std::nullopt;
    }

    // Load scenario entries and build starts/goals sets
    auto entries = SATSolverManager::create_dataframe_from_file(scenario_path);
    if (entries.empty()) {
        std::cerr << "[LNS] No entries found in scenario: " << scenario_path << std::endl;
        return std::nullopt;
    }
    auto sets = SATSolverManager::create_starts_and_goals(entries, num_agents);
    if (sets.empty()) {
        std::cerr << "[LNS] No start/goal sets could be formed from scenario: " << scenario_path << std::endl;
        return std::nullopt;
    }
    if (scenario_index < 0 || scenario_index >= static_cast<int>(sets.size())) {
        std::cerr << "[LNS] scenario_index out of range: " << scenario_index
                  << ", available sets: " << sets.size() << std::endl;
        return std::nullopt;
    }

    problem.starts = sets[scenario_index].first;
    problem.goals = sets[scenario_index].second;

    return problem;
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

        // Build MDDs for this makespan
        auto mdds = SATSolverManager::create_mdds(
            problem.grid, problem.starts, problem.goals, current_max_timesteps, distance_matrices);
        std::cout << "[LNS] Built MDDs for " << mdds.size()
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
        AgentPaths sampled_paths;
        for (size_t agent_id = 0; agent_id < mdds.size(); ++agent_id) {
            const auto& mdd = mdds[agent_id];
            auto path_positions = mdd->sample_random_path(rng);
            std::vector<std::pair<int,int>> as_pairs(path_positions.begin(), path_positions.end());
            sampled_paths[static_cast<int>(agent_id)] = std::move(as_pairs);
        }

        // Detect conflicts (vertex and edge)
        auto [vertex_collisions, edge_collisions] = SATSolverManager::find_all_collisions(sampled_paths);
        std::cout << "[LNS] Sampled solution: vertex collisions=" << vertex_collisions.size()
                  << ", edge collisions=" << edge_collisions.size() << std::endl;

        // If no conflicts, we're done; else proceed to conflict zones and local solves
        if (vertex_collisions.empty() && edge_collisions.empty()) {
            std::cout << "[LNS] Collision-free sampled solution found at makespan " << current_max_timesteps << std::endl;
            break;
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

      
        const int offset = 3; // half the standard conflict zone size
        std::vector<std::pair<int,int>> conflict_points;
        conflict_points.reserve(vertex_collisions.size() + edge_collisions.size() * 2);
        for (const auto& v : vertex_collisions) {
            conflict_points.push_back(std::get<2>(v));
        }
        for (const auto& e : edge_collisions) {
            conflict_points.push_back(std::get<2>(e)); // pos1
            conflict_points.push_back(std::get<3>(e)); // pos2
        }

        int rows = (int)problem.grid.size();
        int cols = (int)problem.grid[0].size();
        std::vector<char> used(conflict_points.size(), 0);
        struct Bucket { int min_row, min_col, max_row, max_col; std::vector<int> indices; std::vector<std::vector<char>> masked_map; };
        std::vector<Bucket> buckets;

        // Rectangular buckets: offset creates square windows around each conflict
        // Each conflict gets a (2*offset+1) x (2*offset+1) square window
        for (size_t i = 0; i < conflict_points.size(); ++i) {
            if (used[i]) continue;
            auto [center_row, center_col] = conflict_points[i];
            int min_row = std::max(0, center_row - offset);
            int min_col = std::max(0, center_col - offset);
            int max_row = std::min(rows - 1, center_row + offset);
            int max_col = std::min(cols - 1, center_col + offset);
            Bucket b{min_row, min_col, max_row, max_col, {}, {}};
            bool changed = true;
            while (changed) {
                changed = false;
                for (size_t j = 0; j < conflict_points.size(); ++j) {
                    if (used[j]) continue;
                    auto [conf_r, conf_c] = conflict_points[j];
                    if (conf_r >= b.min_row && conf_r <= b.max_row && conf_c >= b.min_col && conf_c <= b.max_col) {
                        // add and expand by offset around this conflict
                        b.indices.push_back((int)j);
                        used[j] = 1;
                        int expand_min_row = std::max(0, conf_r - offset);
                        int expand_min_col = std::max(0, conf_c - offset);
                        int expand_max_row = std::min(rows - 1, conf_r + offset);
                        int expand_max_col = std::min(cols - 1, conf_c + offset);
                        int old_min_row = b.min_row, old_min_col = b.min_col, old_max_row = b.max_row, old_max_col = b.max_col;
                        b.min_row = std::min(b.min_row, expand_min_row);
                        b.min_col = std::min(b.min_col, expand_min_col);
                        b.max_row = std::max(b.max_row, expand_max_row);
                        b.max_col = std::max(b.max_col, expand_max_col);
                        if (b.min_row != old_min_row || b.min_col != old_min_col || b.max_row != old_max_row || b.max_col != old_max_col) {
                            changed = true; // bounds expanded; recheck others
                        }
                    }
                }
            }
            // Create masked map for this bucket window
            b.masked_map = mask_map_outside_bounds(problem.grid, b.min_row, b.min_col, b.max_row, b.max_col);
            buckets.push_back(std::move(b));
        }

        // Diamond buckets: offset creates diamond-shaped windows around each conflict
        // A position (r,c) is inside the diamond if it is reachable |r - center_row| + |c - center_col| <= offset
        std::vector<Bucket> diamond_buckets;
        std::vector<char> diamond_used(conflict_points.size(), 0);
        
        for (size_t i = 0; i < conflict_points.size(); ++i) {
            if (diamond_used[i]) continue;
            auto [center_row, center_col] = conflict_points[i];
            
            // Create diamond bucket with radius = offset
            // Use Manhattan distance formula: |r - center_row| + |c - center_col| <= offset
            // this grows each diamond shape individually
            // maybe we can create diamond shapes in one go for each conflict point?
                // as in write all conflicts on the map and then create diamond shapes on the map
                // then look for overlaps and merge them
                // solve biggest first, let its solution cascade or solve at all at the same time
                // if one diamond consumes another we need to disregard the solution that the smaller found individually i think
            std::set<std::pair<int, int>> diamond_positions;
            
            // Add all positions within the diamond
            for (int r = std::max(0, center_row - offset); r <= std::min(rows - 1, center_row + offset); ++r) {
                for (int c = std::max(0, center_col - offset); c <= std::min(cols - 1, center_col + offset); ++c) {
                    // Check if position is inside diamond using Manhattan distance
                    if (std::abs(r - center_row) + std::abs(c - center_col) <= offset) {
                        diamond_positions.insert({r, c});
                    }
                }
            }
            
            // Find bounds of the diamond
            int min_row = rows, min_col = cols, max_row = -1, max_col = -1;
            for (const auto& [r, c] : diamond_positions) {
                min_row = std::min(min_row, r);
                min_col = std::min(min_col, c);
                max_row = std::max(max_row, r);
                max_col = std::max(max_col, c);
            }
            
            // Create bucket and find conflicts within diamond
            Bucket diamond_b{min_row, min_col, max_row, max_col, {}, {}};
            bool diamond_changed = true;
            
            while (diamond_changed) {
                diamond_changed = false;
                for (size_t j = 0; j < conflict_points.size(); ++j) {
                    if (diamond_used[j]) continue;
                    auto [conf_r, conf_c] = conflict_points[j];
                    
                    // Check if conflict is within diamond bounds first (optimization)
                    if (conf_r >= diamond_b.min_row && conf_r <= diamond_b.max_row && 
                        conf_c >= diamond_b.min_col && conf_c <= diamond_b.max_col) {
                        
                        // Check if conflict is actually inside the diamond using Manhattan distance
                        if (std::abs(conf_r - center_row) + std::abs(conf_c - center_col) <= offset) {
                            // Add conflict to diamond bucket
                            diamond_b.indices.push_back((int)j);
                            diamond_used[j] = 1;
                            
                            // Expand diamond around this new conflict
                            auto [new_center_r, new_center_c] = conflict_points[j];
                            
                            // Add new diamond positions around the new conflict
                            for (int r = std::max(0, new_center_r - offset); r <= std::min(rows - 1, new_center_r + offset); ++r) {
                                for (int c = std::max(0, new_center_c - offset); c <= std::min(cols - 1, new_center_c + offset); ++c) {
                                    if (std::abs(r - new_center_r) + std::abs(c - new_center_c) <= offset) {
                                        diamond_positions.insert({r, c});
                                    }
                                }
                            }
                            
                            // Recalculate bounds
                            int old_min_row = diamond_b.min_row, old_min_col = diamond_b.min_col;
                            int old_max_row = diamond_b.max_row, old_max_col = diamond_b.max_col;
                            
                            diamond_b.min_row = std::min(diamond_b.min_row, new_center_r - offset);
                            diamond_b.min_col = std::min(diamond_b.min_col, new_center_c - offset);
                            diamond_b.max_row = std::max(diamond_b.max_row, new_center_r + offset);
                            diamond_b.max_col = std::max(diamond_b.max_col, new_center_c + offset);
                            
                            // Ensure bounds don't exceed map limits
                            diamond_b.min_row = std::max(0, diamond_b.min_row);
                            diamond_b.min_col = std::max(0, diamond_b.min_col);
                            diamond_b.max_row = std::min(rows - 1, diamond_b.max_row);
                            diamond_b.max_col = std::min(cols - 1, diamond_b.max_col);
                            
                            if (diamond_b.min_row != old_min_row || diamond_b.min_col != old_min_col || 
                                diamond_b.max_row != old_max_row || diamond_b.max_col != old_max_col) {
                                diamond_changed = true; // bounds expanded; recheck others
                            }
                        }
                    }
                }
            }
            
            // Create masked map for this diamond bucket
            diamond_b.masked_map = mask_map_outside_bounds(problem.grid, diamond_b.min_row, diamond_b.min_col, diamond_b.max_row, diamond_b.max_col);
            diamond_buckets.push_back(std::move(diamond_b));
        }

        std::cout << "[LNS] Formed " << buckets.size() << " rectangular conflict bucket(s)." << std::endl;
        for (size_t bi = 0; bi < buckets.size(); ++bi) {
            const auto& b = buckets[bi];
            std::cout << "  Rectangular Bucket " << bi << ": window [" << b.min_row << "," << b.min_col << "]..[" << b.max_row << "," << b.max_col
                      << "], conflicts=" << b.indices.size() << std::endl;
        }
        
        std::cout << "[LNS] Formed " << diamond_buckets.size() << " diamond-shaped conflict bucket(s)." << std::endl;
        for (size_t bi = 0; bi < diamond_buckets.size(); ++bi) {
            const auto& b = diamond_buckets[bi];
            std::cout << "  Diamond Bucket " << bi << ": window [" << b.min_row << "," << b.min_col << "]..[" << b.max_row << "," << b.max_col
                      << "], conflicts=" << b.indices.size() << std::endl;
        }

        // Next steps: prepare local problems using these buckets
    }

    return 0;
}
