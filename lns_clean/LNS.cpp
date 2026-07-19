#include "LNS.h"
#include "Load_LNSProblem.h"
#include "Current_Solution.h" //collect_conflicts_meta, create_conflict_map_2D
#include "Local_Zone.h" //build_diamond_buckets, select_most_relevant_bucket
#include "Lazy_SAT_Solve.h" //create_mdds_with_waiting_time
#include "Solve_Local_Zone.h" //solve_local_zone, local_zone_result
#include "../SATSolverManager.h" //find_all_collisions, print_agent_paths
#include "VerificationHelpers.h" //verify_solution_consistency
#include "ExperimentLogger.h"
#include "Metrics.h"
#include "ExperimentLogger.h"

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
#include <chrono>

//95% ->full coverage with full time window last resort solve

//ERRORS
// untailed try before zone expansion

// made this build_segment_mdd --> use instead of build_segment_mdd_with_optional_wait_tail
// every time

// zone expansion growth based on number of attempts 



//TODO:
//incremental sat solve, when solution found -> keep solving after adding new constraints
//minisat can do that, no need to re-solve from scratch
//very initial solution //traffic avoidance sampling from mdds

//final check before makespan increase

//bucket creation, we look for other conflicts in the immediate neighbourhood. Limit that searchto relevant timesteps around the og conflict
//bucket selection, earliest -> most conflicts -> most involved agents -> most involved positions (currently not checking for agents)
//zone expansion by radius is kinda blind, gives more room but is not targetted
//zone expansion to only include actuall reachable positions, not all positions in the diamond shape

//maybe some collisions are reusable to kickstart the zone solve after zone expansion

//Slack allocation strategy too rigid, find a better way to allocate slack




//main function for LNS
//takes map path, scenario path, number of agents, scenario index, use minisat, seed
//returns paths of agents

LNSResult LNS(const SolveRequest& request, const SolverConfig& config) {
    const std::string& map_path = request.map_path;
    const std::string& scenario_path = request.scenario_path;
    const int num_agents = request.num_agents;
    const int scenario_index = request.scenario_index;
    const int seed = config.seed;
    const NeighborhoodVariant variant = config.neighborhood_variant;
    const bool use_minisat = config.backend == SatBackend::MiniSat;
    (void)use_minisat;

    LNSResult solve_result;
    solve_result.seed = seed;
    solve_result.neighborhood_variant = variant;
    solve_result.backend = config.backend;

    const ConfigurationValidation validation =
        validate_solver_configuration(request, config);
    if (!validation.valid) {
        solve_result.status = SolveStatus::InvalidInput;
        solve_result.message = validation.message;
        return solve_result;
    }
    solve_result.search_started = true;
    const SolverDeadline deadline = make_solver_deadline(config);
    
    //Step 1: Load problem and print basic info
    auto problem_loaded = load_problem(map_path, scenario_path, num_agents, scenario_index);
    if (!problem_loaded.has_value()) {
        std::cerr << "[LNS] Failed to load problem" << std::endl;
        solve_result.status = SolveStatus::InvalidInput;
        solve_result.message = "Failed to load map/scenario input";
        return solve_result;
    }
    const auto& problem = problem_loaded.value();
    const NeighborhoodPolicy variant_policy = neighborhood_policy(variant);

    std::cout << "[LNS] Loaded map " << problem.grid.size() << "x"
              << (problem.grid.empty() ? 0 : (int)problem.grid[0].size())
              << ", agents: " << problem.starts.size() << std::endl;
    if (solver_deadline_reached(deadline)) {
        solve_result.status = SolveStatus::Exhausted;
        solve_result.message = "Wall-clock limit reached after input loading";
        return solve_result;
    }

    std::cout << "[LNS] Neighborhood variant: " << variant_policy.canonical_name
              << " (initial radius=" << variant_policy.initial_radius << ")" << std::endl;

    auto& logger = ExperimentLogger::instance();
    std::string experiment_id = logger.start_experiment(map_path, scenario_path, num_agents, scenario_index, seed);
    ExperimentSummaryMetrics summary;
    summary.experiment_id = experiment_id;
    summary.map_path = map_path;
    summary.scenario_path = scenario_path;
    summary.num_agents = num_agents;
    summary.scenario_index = scenario_index;
    summary.seed = seed;
    auto experiment_start = std::chrono::steady_clock::now();

    
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
    std::mt19937 rng(static_cast<unsigned int>(seed));
    std::optional<CurrentSolution> successfull_solution;
    int successful_max_timesteps = -1;
    SolveStatus terminal_status = SolveStatus::Exhausted;
    std::string terminal_message = "No solution found within the configured makespan limit";
    bool fatal_failure = false;
    
    int makespan_attempt_index = 0;
    for (int inc = 0;
         inc <= config.makespan_increase_limit;
         inc += config.makespan_increment, ++makespan_attempt_index) {
        if (solver_deadline_reached(deadline)) {
            terminal_message = "Wall-clock limit reached before makespan attempt";
            break;
        }

        int current_max_timesteps = base_makespan + inc;
        std::cout << "\n[LNS] === Attempt with max_timesteps=" << current_max_timesteps << " ===" << std::endl;
        
        MakespanAttemptMetrics makespan_metrics;
        makespan_metrics.attempt_index = makespan_attempt_index;
        makespan_metrics.makespan = current_max_timesteps;
        auto attempt_start = std::chrono::steady_clock::now();

        //Build MDDs with fastest way to the goal and sample for initial solution
        //this is just for the initial solution, can be improved in the future
        auto mdds = create_mdds_with_waiting_time(
            problem.grid, problem.starts, problem.goals, distance_matrices);
        std::cout << "[LNS] Built MDDs with waiting time structure for " << mdds.size()
                  << " agents at makespan " << current_max_timesteps << std::endl;
        if (mdds.size() != problem.starts.size()) {
            // A partial initial solution is unsafe: missing MDDs used to shift
            // later vector positions onto the wrong agent IDs. Stop this solve
            // attempt instead of constructing a mislabeled path map.
            std::cout << "[LNS] ERROR: Initial MDD construction failed for one or more agents"
                      << std::endl;
            terminal_message = "At least one agent has no constructible initial path";
            break;
        }
        
        //Step 4: create current solution by sampling paths from MDDs
        CurrentSolution current_solution(
            problem.grid.size(), 
            problem.grid[0].size(), 
            current_max_timesteps,
            num_agents,
            problem.starts,
            problem.goals);
        for (const AgentMDD& agent_mdd : mdds) {
            auto path_positions = agent_mdd.mdd->sample_random_path(rng);
            std::vector<std::pair<int,int>> as_pairs(path_positions.begin(), path_positions.end());
            current_solution.agent_paths[agent_mdd.agent_id] = std::move(as_pairs);
        }
        //calculate waiting times for each agent
        current_solution.calculate_waiting_times(problem.goals, current_max_timesteps);
        //pad paths to makespan
        current_solution.pad_paths_to_makespan();  
        
        //verify every agent has their amount of waiting time as goal positions in the end of their path
        for (const auto& [agent_id, path] : current_solution.agent_paths) {
            if (path.back() != current_solution.goals[agent_id]) {
                std::cout << "[LNS] ERROR: Agent " << agent_id << " does not end at the goal position" << std::endl;
            }
            int waiting_time = current_solution.get_waiting_time(agent_id);
            if (path[path.size() - waiting_time - 1] != current_solution.goals[agent_id]) {
                std::cout << "[LNS] ERROR: Agent " << agent_id << " does not end at the goal position with waiting time" << std::endl;
                //print waiting time
                std::cout << "[LNS] Waiting time: " << waiting_time << std::endl;
                //print path
                std::cout << "[LNS] Path (size: " << path.size() << "): ";
                for (const auto& pos : path) {
                    std::cout << "(" << pos.first << ", " << pos.second << ") ";
                }
                std::cout << std::endl;
            }
        }

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
            std::cout << "[LNS] Creating conflict buckets..." << std::endl;
            auto diamond_buckets = build_diamond_buckets_for_earliest_conflicts(
                conflict_meta,
                conflict_map,
                problem.grid,
                variant_policy.initial_radius,
                current_max_timesteps);
            //if multiple buckets, select the most relevant one
            DiamondBucket best_bucket = select_most_relevant_bucket(diamond_buckets);
            if (best_bucket.indices.empty()) {
                std::cout << "[LNS] ERROR: No most relevant bucket found" << std::endl;
                terminal_status = SolveStatus::InvalidState;
                terminal_message = "Conflict selection returned no usable bucket";
                fatal_failure = true;
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
               // problem.grid, best_bucket, conflict_meta, conflict_map, current_solution, offset, current_max_timesteps, rng, experiment_id, inc);
                problem.grid,
                best_bucket,
                conflict_meta,
                conflict_map,
                current_solution,
                variant_policy,
                current_max_timesteps,
                rng,
                experiment_id,
                makespan_attempt_index,
                config,
                deadline);

            for (const auto& zone_metric : local_zone_result.attempt_metrics) {
                makespan_metrics.zones_attempted++;
                if (zone_metric.solved) {
                    makespan_metrics.zones_solved++;
                }
                makespan_metrics.total_waiting_attempts += zone_metric.waiting_attempts;
                makespan_metrics.total_lazy_iterations += zone_metric.total_lazy_iterations;
                makespan_metrics.total_cnf_clauses += zone_metric.total_cnf_clauses;
                makespan_metrics.total_cnf_variables += zone_metric.total_cnf_variables;
                makespan_metrics.total_mdd_build_time_ms += zone_metric.total_mdd_build_time_ms;
                makespan_metrics.total_cnf_build_time_ms += zone_metric.total_cnf_build_time_ms;
                makespan_metrics.total_lazy_wall_time_ms += zone_metric.total_lazy_wall_time_ms;
                makespan_metrics.total_lazy_solver_wall_time_ms += zone_metric.total_lazy_solver_wall_time_ms;
                makespan_metrics.total_lazy_solver_reported_time_ms += zone_metric.total_lazy_solver_reported_time_ms;
            }

            //Step 8: Update the current solution with the local zone result if found
            if (local_zone_result.status == SolveStatus::InvalidInput ||
                local_zone_result.status == SolveStatus::InvalidState) {
                terminal_status = local_zone_result.status;
                terminal_message = local_zone_result.message;
                fatal_failure = true;
                break;
            }
            if (local_zone_result.solved()) {
                std::cout << "[LNS] Successfully solved local zone" << std::endl;
                //integrate local zone result into current solution
                //solution is updated in the waiting time solve
                //loop back to step 5
            }
            //if impossible to solve, increase makespan
            if (!local_zone_result.solved()) {
                std::cout << "[LNS] Impossible to solve local zone with current makespan: " << current_max_timesteps << std::endl;
                conflicts_remain = true;
                break;
            }
        }

        makespan_metrics.solved = !conflicts_remain;
        makespan_metrics.attempt_wall_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - attempt_start).count() / 1000.0;
        logger.log_makespan_attempt(experiment_id, makespan_metrics);
        summary.total_cnf_clauses += makespan_metrics.total_cnf_clauses;
        summary.total_cnf_variables += makespan_metrics.total_cnf_variables;
        summary.total_mdd_build_time_ms += makespan_metrics.total_mdd_build_time_ms;
        summary.total_cnf_build_time_ms += makespan_metrics.total_cnf_build_time_ms;
        summary.total_lazy_wall_time_ms += makespan_metrics.total_lazy_wall_time_ms;
        summary.total_lazy_solver_wall_time_ms += makespan_metrics.total_lazy_solver_wall_time_ms;
        summary.total_lazy_solver_reported_ms += makespan_metrics.total_lazy_solver_reported_time_ms;
        if (makespan_metrics.solved && summary.makespan_success == -1) {
            summary.makespan_success = current_max_timesteps;
            summary.solved = true;
        }

        if (fatal_failure) {
            std::cout << "[LNS] Stopping after fatal status "
                      << solve_status_name(terminal_status) << ": "
                      << terminal_message << std::endl;
            break;
        }

        if (!conflicts_remain) {
            std::cout << "[LNS] All conflicts resolved" << std::endl;
            successfull_solution = std::move(current_solution);
            successful_max_timesteps = current_max_timesteps;
            break;
        } else {
            std::cout << "[LNS] Increasing makespan..." << std::endl;
        }
    }
    // Step 9: validate the complete solution before reporting success. The
    // verifier is the final trust boundary for all preceding repair logic.
    summary.total_runtime_ms = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - experiment_start).count() / 1000.0;
    solve_result.runtime_ms = summary.total_runtime_ms;

    if (!successfull_solution.has_value()) {
        summary.solved = false;
        summary.makespan_success = -1;
        logger.log_experiment_summary(summary);
        std::cout << "[LNS] ERROR: No successful solution found" << std::endl;
        solve_result.status = terminal_status;
        solve_result.message = terminal_message;
        return solve_result;
    }

    const bool valid_solution = verify_solution_consistency(
        successfull_solution->agent_paths, problem.starts, problem.goals, problem.grid);
    if (!valid_solution) {
        summary.solved = false;
        summary.makespan_success = -1;
        logger.log_experiment_summary(summary);
        std::cout << "[LNS] ERROR: Final solution verification failed" << std::endl;
        SATSolverManager::print_agent_paths(successfull_solution->agent_paths);
        // Fail closed: callers must never receive a non-empty invalid path map.
        solve_result.status = SolveStatus::InvalidState;
        solve_result.message = "Final solution failed independent verification";
        return solve_result;
    }

    logger.log_experiment_summary(summary);
    std::cout << "[LNS] Collision-free verified solution found at makespan "
              << successful_max_timesteps << std::endl;
    solve_result.status = SolveStatus::Solved;
    solve_result.paths = std::move(successfull_solution->agent_paths);
    solve_result.makespan = successful_max_timesteps;
    solve_result.message = "Verified collision-free solution";
    return solve_result;
}

LNSResult LNS(
    const std::string& map_path,
    const std::string& scenario_path,
    int num_agents,
    int scenario_index,
    bool use_minisat,
    int seed,
    NeighborhoodVariant variant) {
    SolveRequest request{
        map_path, scenario_path, num_agents, scenario_index};
    SolverConfig config;
    config.seed = seed;
    config.neighborhood_variant = variant;
    config.backend =
        use_minisat ? SatBackend::MiniSat : SatBackend::ProbSat;
    return LNS(request, config);
}
