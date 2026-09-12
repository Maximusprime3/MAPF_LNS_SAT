#include "lnssat/Logging.h"
#include "lnssat/SATSolverManager.h" //EdgeAgentMap
#include "lnssat/Lazy_SAT_Solve.h"
#include <unordered_map>
#include <vector>
#include <tuple>
#include <set>
#include <iostream>
#include <utility>
#include <chrono>





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
        PositionAgentMap position_agents;
        
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
std::vector<AgentMDD> create_mdds_with_waiting_time(
    const std::vector<std::vector<char>>& grid,
    const std::vector<std::pair<int,int>>& starts,
    const std::vector<std::pair<int,int>>& goals,
    const std::vector<std::map<std::pair<int,int>, int>>& distance_matrices, SolverDeadline deadline) {
    
    lnssat::debug_log() << "[SAT] Creating MDDs with shortest paths + waiting time..." << std::endl;
    
    // All parallel inputs must describe the same ordered set of agents. Check
    // this before indexing so malformed input fails without undefined behavior.
    if (starts.size() != goals.size() || starts.size() != distance_matrices.size()) {
        std::cerr << "[SAT] ERROR: Initial MDD inputs have inconsistent agent counts"
                  << std::endl;
        return {};
    }

    std::vector<AgentMDD> mdds;
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
        MDDConstructor constructor(grid, start, goal, shortest_path_length, {}, deadline);
        auto mdd = constructor.construct_mdd();
        
        if (!mdd) {
            std::cerr << "[SAT] ERROR: Failed to create MDD for Agent " << agent_id << std::endl;
            continue;
        }
        
        // Preserve the source ID even if an earlier agent failed. Consumers
        // must use this field rather than treating vector position as identity.
        mdds.push_back({static_cast<int>(agent_id), std::move(mdd)});
    }
    
    lnssat::debug_log() << "[SAT] Created " << mdds.size() << " MDDs with waiting time structure" << std::endl;
    return mdds;
}


namespace {

void accumulate_statistics(
    SatStatistics& total,
    const SatStatistics& current) {
    total.decisions += current.decisions;
    total.propagations += current.propagations;
    total.solve_time_seconds += current.solve_time_seconds;
}

}  // namespace

SatIterationResult solve_sat_iteration(
    SatSolver& solver,
    const std::vector<SatClause>& accumulated_clauses,
    std::size_t& loaded_clause_count,
    const SatAssumptions* assumptions,
    bool reset_before_solve,
    const SolverDeadline& deadline) {
    SatIterationResult aggregate;
    solver.set_deadline(deadline);
    auto expired = [&]() {
        if (!solver_deadline_reached(deadline)) return false;
        aggregate.kind = SatResultKind::Interrupted;
        aggregate.diagnostic = "Wall-clock limit reached during SAT iteration";
        aggregate.model.clear();
        return true;
    };

    auto call = [&](bool reset,
                    const SatAssumptions* call_assumptions) {
        const auto start = std::chrono::steady_clock::now();
        if (expired()) return;
        if (reset) {
            aggregate.reset_solver = true;
            const SatOperationResult reset_result = solver.reset();
            if (expired()) return;
            if (!reset_result.ok) {
                aggregate.kind = SatResultKind::Error;
                aggregate.diagnostic = reset_result.diagnostic;
                return;
            }
            loaded_clause_count = 0;
        }

        if (loaded_clause_count > accumulated_clauses.size()) {
            aggregate.kind = SatResultKind::Error;
            aggregate.diagnostic =
                "SAT clause prefix exceeds accumulated formula";
            return;
        }

        if (loaded_clause_count < accumulated_clauses.size()) {
            std::vector<SatClause> appended(
                accumulated_clauses.begin() +
                    static_cast<std::ptrdiff_t>(loaded_clause_count),
                accumulated_clauses.end());
            const SatOperationResult add_result =
                solver.add_clauses(appended);
            if (expired()) return;
            if (!add_result.ok) {
                aggregate.kind = SatResultKind::Error;
                aggregate.diagnostic = add_result.diagnostic;
                return;
            }
            loaded_clause_count = accumulated_clauses.size();
        }

        SatSolveResult result;
        if (call_assumptions != nullptr) {
            aggregate.used_assumptions = true;
            result = solver.solve(*call_assumptions);
        } else {
            result = solver.solve();
        }
        const auto end = std::chrono::steady_clock::now();

        ++aggregate.solver_calls;
        aggregate.solver_wall_time_ms +=
            std::chrono::duration_cast<std::chrono::microseconds>(
                end - start).count() /
            1000.0;
        accumulate_statistics(
            aggregate.statistics, result.statistics);
        if (expired()) return;
        aggregate.kind = result.kind;
        aggregate.diagnostic = result.diagnostic;
        if (result.kind == SatResultKind::Sat) {
            aggregate.model = solver.model();
        }
    };

    call(reset_before_solve, assumptions);
    if (assumptions != nullptr &&
        aggregate.kind == SatResultKind::Unsat && !expired()) {
        call(true, nullptr);
    }

    return aggregate;
}

//Solves a local zone with SAT
LazySolveResult lazy_SAT_solve(
    SatSolver& solver,
    CNF& local_cnf,
    CNFConstructor& cnf_constructor,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int end_t,
    int max_iterations,
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& initial_vertex_collisions,
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& initial_edge_collisions,
    const SolverDeadline& deadline) {

    lnssat::debug_log() << "[SAT] Start solving CNF: " << local_cnf.get_clauses().size() << " clauses and "
              << (cnf_constructor.get_next_variable_id() - 1) << " variables" << std::endl;

    for (const auto& [agent_id, entry_exit_time] : local_entry_exit_time) {
        lnssat::debug_log() << "[SAT] Agent " << agent_id << " entry time: " << entry_exit_time.first << " exit time: " << entry_exit_time.second << std::endl;
    }
    lnssat::debug_log() << std::endl;
    lnssat::debug_log() << "[SAT] Start time: " << start_t << " End time: " << end_t << std::endl;
    lnssat::debug_log() << std::endl;

    auto set_to_vector_vertex = [](const std::set<std::tuple<int, int, std::pair<int,int>, int>>& s) {
        return std::vector<std::tuple<int, int, std::pair<int,int>, int>>(s.begin(), s.end());
    };
    auto set_to_vector_edge = [](const std::set<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& s) {
        return std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>(s.begin(), s.end());
    };

    std::set<std::tuple<int, int, std::pair<int,int>, int>> discovered_vertex_collisions_set;
    std::set<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> discovered_edge_collisions_set;
    std::set<std::tuple<int, int, std::pair<int,int>, int>> latest_discovered_vertex_collisions;
    std::set<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> latest_discovered_edge_collisions;
    if (!initial_vertex_collisions.empty()) {
        discovered_vertex_collisions_set.insert(initial_vertex_collisions.begin(), initial_vertex_collisions.end());
        cnf_constructor.add_collision_clauses_to_cnf(local_cnf, initial_vertex_collisions);
    }
    if (!initial_edge_collisions.empty()) {
        discovered_edge_collisions_set.insert(initial_edge_collisions.begin(), initial_edge_collisions.end());
        cnf_constructor.add_edge_collision_clauses_to_cnf(local_cnf, initial_edge_collisions);
    }

    bool solution_found = false;
    SolveStatus final_status = SolveStatus::Exhausted;
    std::string final_message = "Lazy SAT iteration limit reached";
    bool first_iteration = true;
    int iteration = 0;
    std::size_t loaded_clause_count = 0;
    std::unordered_map<int, std::vector<std::pair<int,int>>> final_local_paths;
    SatAssumptions previous_path_assumptions;

    LazySolveRunMetrics run_metrics;
    auto run_start = std::chrono::steady_clock::now();

    while (!solution_found && iteration < max_iterations) {
        iteration++;
        lnssat::debug_log() << "[SAT] Solving local zone with SAT iteration " << iteration << "..." << std::endl;
        lnssat::debug_log() << "[SAT] Local CNF: " << local_cnf.get_clauses().size() << " clauses " << std::endl;

        LazySatIterationMetrics iteration_metrics;
        iteration_metrics.iteration = iteration;
        iteration_metrics.clause_count_before = static_cast<int>(local_cnf.get_clauses().size());
        iteration_metrics.variable_count = local_cnf.count_variables();
        auto iteration_start = std::chrono::steady_clock::now();

        const SatAssumptions* assumptions_ptr = nullptr;
        if (!first_iteration && !previous_path_assumptions.literals.empty()) {
            assumptions_ptr = &previous_path_assumptions;
        }

        const SatIterationResult sat_result = solve_sat_iteration(
            solver,
            local_cnf.get_clauses(),
            loaded_clause_count,
            assumptions_ptr,
            first_iteration, deadline);
        first_iteration = false;

        iteration_metrics.solver_wall_time_ms =
            sat_result.solver_wall_time_ms;
        iteration_metrics.solver_reported_time_ms =
            seconds_to_milliseconds(
                sat_result.statistics.solve_time_seconds);
        iteration_metrics.solver_calls = sat_result.solver_calls;
        iteration_metrics.used_assumptions =
            sat_result.used_assumptions;
        iteration_metrics.reset_solver =
            sat_result.reset_solver;
        iteration_metrics.satisfiable =
            sat_result.kind == SatResultKind::Sat;
        iteration_metrics.num_decisions =
            sat_result.statistics.decisions;
        iteration_metrics.num_propagations =
            sat_result.statistics.propagations;

        if (sat_result.kind != SatResultKind::Sat) {
            iteration_metrics.total_vertex_collisions = static_cast<int>(discovered_vertex_collisions_set.size());
            iteration_metrics.total_edge_collisions = static_cast<int>(discovered_edge_collisions_set.size());
            iteration_metrics.total_clauses_after = static_cast<int>(local_cnf.get_clauses().size());
            iteration_metrics.clauses_added = iteration_metrics.total_clauses_after - iteration_metrics.clause_count_before;
            auto iteration_end = std::chrono::steady_clock::now();
            iteration_metrics.iteration_wall_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(iteration_end - iteration_start).count() / 1000.0;
            run_metrics.iterations.push_back(iteration_metrics);
            run_metrics.total_solver_wall_time_ms += sat_result.solver_wall_time_ms;
            run_metrics.total_solver_reported_time_ms += iteration_metrics.solver_reported_time_ms;
            if (sat_result.kind == SatResultKind::Interrupted) {
                final_status = SolveStatus::Exhausted;
                final_message = "Wall-clock limit reached during lazy SAT solving";
            } else if (sat_result.kind == SatResultKind::Error) {
                final_status = SolveStatus::InvalidState;
                final_message = "SAT backend failure: " +
                                (sat_result.diagnostic.empty()
                                     ? std::string("unknown error")
                                     : sat_result.diagnostic);
            } else {
                final_status = SolveStatus::Exhausted;
                final_message = "Local CNF is unsatisfiable";
            }
            break;
        }

        std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths;
        try {
            local_paths =
                cnf_constructor.cnf_assignment_to_paths(
                    sat_result.model);
        } catch (const std::exception& error) {
            final_status = SolveStatus::InvalidState;
            final_message =
                std::string("SAT model extraction failed: ") +
                error.what();
            break;
        }

        auto new_collisions = check_vertex_collisions_local(local_paths, local_entry_exit_time, start_t, end_t);
        auto new_edge_collisions = check_edge_collisions_local(local_paths, local_entry_exit_time, start_t, end_t);

        discovered_vertex_collisions_set.insert(new_collisions.begin(), new_collisions.end());
        discovered_edge_collisions_set.insert(new_edge_collisions.begin(), new_edge_collisions.end());
        latest_discovered_vertex_collisions.insert(new_collisions.begin(), new_collisions.end());
        latest_discovered_edge_collisions.insert(new_edge_collisions.begin(), new_edge_collisions.end());

        iteration_metrics.new_vertex_collisions = static_cast<int>(new_collisions.size());
        iteration_metrics.new_edge_collisions = static_cast<int>(new_edge_collisions.size());
        iteration_metrics.total_vertex_collisions = static_cast<int>(discovered_vertex_collisions_set.size());
        iteration_metrics.total_edge_collisions = static_cast<int>(discovered_edge_collisions_set.size());

        lnssat::debug_log() << "[SAT] Found " << new_collisions.size() << " vertex collisions and "
                  << new_edge_collisions.size() << " edge collisions" << std::endl;

        if (solver_deadline_reached(deadline)) {
            final_message = "Wall-clock limit reached after SAT model validation";
            break;
        }
        if (new_collisions.empty() && new_edge_collisions.empty()) {
            solution_found = true;
            final_status = SolveStatus::Solved;
            final_message = "Collision-free local SAT solution";
            final_local_paths = std::move(local_paths);
            lnssat::debug_log() << "[SAT] Found collision-free local solution!" << std::endl;
        } else {
            cnf_constructor.add_collision_clauses_to_cnf(local_cnf, new_collisions);
            cnf_constructor.add_edge_collision_clauses_to_cnf(local_cnf, new_edge_collisions);
            try {
                previous_path_assumptions.literals = cnf_constructor.partial_assignment_from_paths(local_paths);
            } catch (const std::exception& error) {
                final_status = SolveStatus::InvalidState;
                final_message = std::string("SAT path assumptions failed: ") + error.what();
                break;
            }
            lnssat::debug_log() << "[SAT] Adding collision clauses and solving again..." << std::endl;
        }
        iteration_metrics.total_clauses_after = static_cast<int>(local_cnf.get_clauses().size());
        iteration_metrics.clauses_added = iteration_metrics.total_clauses_after - iteration_metrics.clause_count_before;
        auto iteration_end = std::chrono::steady_clock::now();
        iteration_metrics.iteration_wall_time_ms =
            std::chrono::duration_cast<std::chrono::microseconds>(iteration_end - iteration_start).count() / 1000.0;

        run_metrics.iterations.push_back(iteration_metrics);
        run_metrics.total_solver_wall_time_ms += sat_result.solver_wall_time_ms;
        run_metrics.total_solver_reported_time_ms += iteration_metrics.solver_reported_time_ms;
    }
    auto run_end = std::chrono::steady_clock::now();
    run_metrics.total_wall_time_ms =
        std::chrono::duration_cast<std::chrono::microseconds>(run_end - run_start).count() / 1000.0;
    run_metrics.final_clause_count = static_cast<int>(local_cnf.get_clauses().size());
    run_metrics.final_variable_count = local_cnf.count_variables();
    run_metrics.solved = solution_found;
    if (!solution_found) {
        lnssat::debug_log() << "[SAT] Local solve ended with status "
                  << solve_status_name(final_status) << ": " << final_message << std::endl;
    }

    LazySolveResult result;
    result.status = final_status;
    result.message = std::move(final_message);
    result.local_paths = std::move(final_local_paths);
    result.local_entry_exit_time = local_entry_exit_time;
    result.discovered_vertex_collisions = set_to_vector_vertex(discovered_vertex_collisions_set);
    result.discovered_edge_collisions = set_to_vector_edge(discovered_edge_collisions_set);
    result.latest_discovered_vertex_collisions = set_to_vector_vertex(latest_discovered_vertex_collisions);
    result.latest_discovered_edge_collisions = set_to_vector_edge(latest_discovered_edge_collisions);
    result.metrics = std::move(run_metrics);
    return result;
}

LazySolveResult lazy_SAT_solve(
    CNF& local_cnf,
    CNFConstructor& cnf_constructor,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int end_t,
    int max_iterations,
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& initial_vertex_collisions,
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& initial_edge_collisions,
    const SolverDeadline& deadline) {
    auto solver = make_sat_solver();
    return lazy_SAT_solve(
        *solver,
        local_cnf,
        cnf_constructor,
        local_entry_exit_time,
        start_t,
        end_t,
        max_iterations,
        initial_vertex_collisions,
        initial_edge_collisions, deadline);
}
