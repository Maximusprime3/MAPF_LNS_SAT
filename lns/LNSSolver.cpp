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
            // - For each zone, create masked map and local agent subsets
            // - Construct local MDDs and CNF for each conflict subproblem
            // - Solve each conflict subproblem with SAT/ProbSAT first if fails then CDCL; 
                // - solve in parallel (whatabout kasskades) vs serial (safe and easy)
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

        //rectangle buckets
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

        //diamond buckets offset is not used to create rectangle windows, 
        // but is used to create shaped diamond windows 
        // aka all reachable positions from the center with in the number of timesteps specified by offset
        for (size_t i = 0; i < conflict_points.size(); ++i) {
            if (used[i]) continue;
            auto [center_row, center_col] = conflict_points[i];
            int min_row = std::max(0, center_row - offset);
            int min_col = std::max(0, center_col - offset);
            int max_row = std::min(rows - 1, center_row + offset);
            int max_col = std::min(cols - 1, center_col + offset);
            //the bucket is a diamond window with the center at the conflict point and a radius of the offset
            //the bucket extends in all four directions from the center to the offset resulting in a cross shape
            

        }

        std::cout << "[LNS] Formed " << buckets.size() << " conflict bucket(s)." << std::endl;
        for (size_t bi = 0; bi < buckets.size(); ++bi) {
            const auto& b = buckets[bi];
            std::cout << "  Bucket " << bi << ": window [" << b.min_row << "," << b.min_col << "]..[" << b.max_row << "," << b.max_col
                      << "], conflicts=" << b.indices.size() << std::endl;
        }

        // Next steps: prepare local problems using these buckets
    }

    return 0;
}
