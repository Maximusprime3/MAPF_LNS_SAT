#ifndef LNS_LAZY_SAT_SOLVE_H
#define LNS_LAZY_SAT_SOLVE_H

#include "../SATSolverManager.h"
#include "Metrics.h"
#include "SatSolver.h"
#include "SolveStatus.h"
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
 * Contains the bounded SAT outcome, final local paths if solved,
 * and sets of discovered vertex/edge collisions (both cumulative and latest
 * before UNSAT) expressed in global timesteps.
 */
struct LazySolveResult {
    SolveStatus status = SolveStatus::Exhausted;
    std::string message;
    std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths;
    std::unordered_map<int, std::pair<int,int>> local_entry_exit_time;
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> discovered_vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> discovered_edge_collisions;
    // latest collisions that were discovered before UNSAT
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> latest_discovered_vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> latest_discovered_edge_collisions;
    LazySolveRunMetrics metrics;

    bool solved() const {
        return status == SolveStatus::Solved;
    }
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
 * @return LazySolveResult with status, paths, diagnostics, and collision sets.
 */
struct SatIterationResult {
    SatResultKind kind = SatResultKind::Error;
    std::string diagnostic;
    std::vector<int> model;
    SatStatistics statistics;
    double solver_wall_time_ms = 0.0;
    int solver_calls = 0;
    bool used_assumptions = false;
    bool reset_solver = false;
};

SatIterationResult solve_sat_iteration(
    SatSolver& solver,
    const std::vector<SatClause>& accumulated_clauses,
    std::size_t& loaded_clause_count,
    const SatAssumptions* assumptions,
    bool reset_before_solve,
    const SolverDeadline& deadline = {});

LazySolveResult lazy_SAT_solve(
    SatSolver& solver,
    CNF& local_cnf,
    CNFConstructor& cnf_constructor,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int end_t,
    int max_iterations = 1000,
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& initial_vertex_collisions = {},
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& initial_edge_collisions = {},
    const SolverDeadline& deadline = {}
);

// Compatibility entry point for low-level callers. Production orchestration
// creates a fresh solver explicitly and injects it through the overload above.
LazySolveResult lazy_SAT_solve(
    CNF& local_cnf,
    CNFConstructor& cnf_constructor,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int end_t,
    int max_iterations = 1000,
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& initial_vertex_collisions = {},
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& initial_edge_collisions = {},
    const SolverDeadline& deadline = {}
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
 * @param distance_matrices  Per-agent distance maps.
 * @return Ordered records containing both the original agent ID and its MDD.
 *
 * The ID is stored explicitly because failed construction must not cause later
 * vector elements to be mistaken for a different agent.
 */
struct AgentMDD {
    int agent_id;
    std::shared_ptr<MDD> mdd;
};

std::vector<AgentMDD> create_mdds_with_waiting_time(
    const std::vector<std::vector<char>>& grid,
    const std::vector<std::pair<int,int>>& starts,
    const std::vector<std::pair<int,int>>& goals,
    const std::vector<std::map<std::pair<int,int>, int>>& distance_matrices, SolverDeadline deadline = {});

#endif // LNS_LAZY_SAT_SOLVE_H
