#ifndef LNS_LAZY_SAT_SOLVE_H
#define LNS_LAZY_SAT_SOLVE_H

#include "../SATSolverManager.h"
#include <unordered_map>
#include <vector>
#include <tuple>
#include <set>
#include <utility>
#include <memory>
#include <map>
// Forward declarations for MDD utilities used by helper APIs
class MDD;
class MDDConstructor;



/**
 * @brief Result of lazy SAT solving for a local zone.
 *
 * Contains whether a solution was found, final local paths if so,
 * and sets of discovered vertex/edge collisions (both cumulative and latest
 * before UNSAT) expressed in global timesteps.
 */
struct LazySolveResult {
    bool solution_found;
    std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths;
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> discovered_vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> discovered_edge_collisions;
    // latest collisions that were discovered before UNSAT
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> latest_discovered_vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> latest_discovered_edge_collisions;
};

/**
 * @brief Solve a local zone using lazy SAT with iterative collision learning.
 *
 * Repeatedly solves the CNF, translates assignments to paths, detects
 * vertex/edge collisions in the global time window [start_t, end_t],
 * and augments the CNF with corresponding prevention clauses until
 * collision-free or iteration limit is reached.
 *
 * @param local_cnf  CNF to solve (mutated with added clauses).
 * @param cnf_constructor  Translator utilities for CNF and assignments.
 * @param local_entry_exit_time  Agent -> (entry_t, exit_t) into the zone.
 * @param start_t  Global start timestep of the zone.
 * @param end_t  Global end timestep of the zone (inclusive).
 * @param max_iterations  Iteration limit for the lazy loop.
 * @param initial_vertex_collisions  Seed vertex collisions to enforce.
 * @param initial_edge_collisions  Seed edge collisions to enforce.
 * @return LazySolveResult with solution flag, paths, and collision sets.
 */
LazySolveResult lazy_SAT_solve(
    CNF& local_cnf,
    CNFConstructor& cnf_constructor,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int end_t,
    int max_iterations = 1000,
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& initial_vertex_collisions = {},
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& initial_edge_collisions = {}
);

/**
 * @brief Check vertex collisions for local paths within [start_t, end_t].
 * @param local_paths  Agent -> sequence of positions within the zone.
 * @param local_entry_exit_time  Agent -> (entry_t, exit_t) global times.
 * @param start_t  Global start timestep (inclusive).
 * @param end_t  Global end timestep (inclusive).
 * @return (agent1, agent2, position, global_timestep) collisions.
 */
std::vector<std::tuple<int, int, std::pair<int,int>, int>> check_vertex_collisions_local(
    const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_paths,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int end_t);

/**
 * @brief Check edge swap collisions for local paths within [start_t, end_t].
 * @param local_paths  Agent -> sequence of positions within the zone.
 * @param local_entry_exit_time  Agent -> (entry_t, exit_t) global times.
 * @param start_t  Global start timestep (inclusive).
 * @param end_t  Global end timestep (inclusive).
 * @return (agent1, agent2, from(pos1), to(pos2), global_timestep) edge collisions.
 */
std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> check_edge_collisions_local(
    const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_paths,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int end_t);

/**
 * @brief Create MDDs for agents with shortest paths and waiting at goal.
 * @param grid  Map grid.
 * @param starts  Agent start positions.
 * @param goals  Agent goal positions.
 * @param makespan  Planning horizon (may be used by constructor logic).
 * @param distance_matrices  Per-agent distance maps.
 * @return Vector of shared MDDs (one per agent that succeeded).
 */
std::vector<std::shared_ptr<MDD>> create_mdds_with_waiting_time(
    const std::vector<std::vector<char>>& grid,
    const std::vector<std::pair<int,int>>& starts,
    const std::vector<std::pair<int,int>>& goals,
    int makespan,
    const std::vector<std::map<std::pair<int,int>, int>>& distance_matrices);

#endif // LNS_LAZY_SAT_SOLVE_H


