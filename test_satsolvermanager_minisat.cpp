#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <filesystem> // For directory creation

// Include our MAPF components
#include "mdd/MDDConstructor.h"
#include "mdd/MDD.h"
#include "mdd/MDDNode.h"
#include "cnf/CNFConstructor.h"
#include "cnf/CNF.h"
#include "SATSolverManager.h"

// Type aliases for better readability (same as in SATSolverManager.h)
using AgentMDDMap = std::unordered_map<int, std::shared_ptr<MDD>>;  // Maps agent_id -> MDD
using AgentPaths = std::unordered_map<int, std::vector<std::pair<int, int>>>;  // Maps agent_id -> path
using AgentPathsWithPosition = std::unordered_map<int, std::vector<MDDNode::Position>>;  // Maps agent_id -> path with Position type
using VariableMap = std::unordered_map<std::tuple<int, MDDNode::Position, int>, int>;  // Maps (agent_id, position, timestep) -> variable_id
using PositionDistanceMap = std::unordered_map<MDDNode::Position, int, pair_hash>;  // Maps position -> distance from goal
using PositionAgentMap = std::unordered_map<std::pair<int, int>, std::vector<int>, pair_hash>;  // Maps position -> list of agents
using EdgeAgentMap = std::unordered_map<std::pair<std::pair<int, int>, std::pair<int, int>>, std::vector<int>, edge_hash>;  // Maps edge -> list of agents
using NodeMapping = std::unordered_map<const MDDNode*, std::shared_ptr<MDDNode>>;  // Maps old node -> new node

// Helper function to load map from file (same as in test_orchestration.cpp)
std::vector<std::vector<char>> load_map(const std::string& map_path) {
    std::ifstream file(map_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open map file: " + map_path);
    }
    
    std::string line;
    std::vector<std::vector<char>> grid;
    
    // Skip header lines
    while (std::getline(file, line)) {
        if (line == "map") {
            break;
        }
    }
    
    // Read the actual map
    while (std::getline(file, line)) {
        if (line.empty()) break;
        std::vector<char> row;
        for (char c : line) {
            if (c == '.' || c == '@' || c == 'T' || c == 'S' || c == 'W' || c == 'G') {
                row.push_back(c);
            }
        }
        if (!row.empty()) {
            grid.push_back(row);
        }
    }
    
    return grid;
}

// Helper function to load scenario and extract agent starts/goals (same as in test_orchestration.cpp)
std::vector<std::pair<MDDNode::Position, MDDNode::Position>> 
load_scenario_agents(const std::string& scenario_path, int num_agents) {
    std::ifstream file(scenario_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open scenario file: " + scenario_path);
    }
    
    std::string line;
    std::vector<std::pair<MDDNode::Position, MDDNode::Position>> agents;
    
    // Skip version line
    std::getline(file, line);
    
    // Read agent data
    int agent_count = 0;
    while (std::getline(file, line) && agent_count < num_agents) {
        std::istringstream iss(line);
        int agent_id, map_w, map_h, start_x, start_y, goal_x, goal_y;
        std::string map_name;
        double distance;
        
        if (iss >> agent_id >> map_name >> map_w >> map_h >> start_x >> start_y >> goal_x >> goal_y >> distance) {
            agents.push_back({{start_x, start_y}, {goal_x, goal_y}});
            agent_count++;
        }
    }
    
    return agents;
}

int main() {
    // Ensure the data directory exists for logging, before any other code
    try {
        if (!std::filesystem::create_directories("data") && !std::filesystem::exists("data")) {
            std::cerr << "ERROR: Could not create or access 'data' directory for logging." << std::endl;
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Exception while creating 'data' directory: " << e.what() << std::endl;
        return 1;
    }
    try {
        std::cout << "=== MAPF LNS SAT with SATSolverManager - MiniSAT Iterative Loop Test ===" << std::endl;
        
        // Configuration
        const std::string map_path = "mapf-map/empty-16-16.map";
        const std::string scenario_path = "mapf-scen-even/scen-even/empty-16-16-even-1.scen";
        const int num_agents = 3;  // Increased to 3 agents
        const int initial_max_timesteps = 2;  // Start with 2 timesteps
        const int max_timestep_increase = 10;  // Maximum additional timesteps
        const long long seed = 42;
        
        std::cout << "Loading map from: " << map_path << std::endl;
        auto grid = load_map(map_path);
        std::cout << "Map loaded: " << grid.size() << "x" << grid[0].size() << std::endl;
        
        std::cout << "Loading scenario from: " << scenario_path << std::endl;
        auto agents = load_scenario_agents(scenario_path, num_agents);
        // Manually override the goals for testing - 3 agents with complex interactions
        if (agents.size() >= 3) {
            // Agent 0: start and goal closer together
            agents[0].first = {1, 1};
            agents[0].second = {1, 3};  // Reduced from (1,4) to (1,3)
            // Agent 1: start and goal closer together, crossing paths
            agents[1].first = {1, 3};   // Start at agent 0's goal
            agents[1].second = {1, 1};  // Goal at agent 0's start
            // Agent 2: new agent with start (1,4) and goal (1,0)
            agents[2].first = {1, 4};   // Start at (1,4)
            agents[2].second = {1, 0};  // Goal at (1,0) - crosses through the middle
        }
        std::cout << "Loaded " << agents.size() << " agents" << std::endl;
        
        // Print agent information
        for (size_t i = 0; i < agents.size(); ++i) {
            std::cout << "Agent " << i << ": Start(" << agents[i].first.first << "," << agents[i].first.second 
                      << ") -> Goal(" << agents[i].second.first << "," << agents[i].second.second << ")" << std::endl;
        }
        
        // Convert agents to starts and goals format
        std::vector<std::pair<int, int>> starts, goals;
        for (const auto& agent : agents) {
            starts.push_back(agent.first);
            goals.push_back(agent.second);
        }
        
        // Compute initial max timesteps and distance matrices
        auto [distance_matrices, computed_max_timesteps] = SATSolverManager::compute_max_timesteps(grid, starts, goals);
        int current_max_timesteps = std::max(computed_max_timesteps, initial_max_timesteps);
        std::cout << "Initial computed max timesteps: " << computed_max_timesteps << ", starting with: " << current_max_timesteps << std::endl;
        
        // Variables to accumulate for logging
        std::vector<double> solver_times_per_iter;
        std::vector<int> decisions_per_iter;
        std::vector<int> propagations_per_iter;
        std::vector<int> tries_per_iter;
        std::vector<int> collisions_per_iter;
        double total_solver_time = 0.0;
        double total_cnf_build_time = 0.0;
        int cnf_vars_start = 0, cnf_clauses_start = 0;
        int cnf_vars_end = 0, cnf_clauses_end = 0;
        std::string solver_used = "MiniSAT";
        std::string status = "UNKNOWN";
        int num_agents_logged = num_agents;
        std::string map_name_logged = map_path;
        

        // Main iterative loop
        for (int timestep_increase = 0; timestep_increase <= max_timestep_increase; ++timestep_increase) {
            auto timestep_start = std::chrono::high_resolution_clock::now();
            std::cout << "\n" << std::string(60, '=') << std::endl;
            std::cout << "=== ITERATION " << (timestep_increase + 1) << " (Timesteps: " << current_max_timesteps << ") ===" << std::endl;
            std::cout << std::string(60, '=') << std::endl;
            
            // Step 1: Create MDDs for current timesteps
            std::cout << "\n=== Creating MDDs with " << current_max_timesteps << " timesteps ===" << std::endl;
            auto mdds = SATSolverManager::create_mdds(grid, starts, goals, current_max_timesteps, distance_matrices);
            
            // Verify MDDs are valid
            for (size_t i = 0; i < mdds.size(); ++i) {
                if (mdds[i]->is_broken(current_max_timesteps)) {
                    std::cerr << "ERROR: MDD for Agent " << i << " is broken!" << std::endl;
                    return 1;
                }
                std::cout << "Agent " << i << " MDD has " << mdds[i]->levels.size() << " timesteps" << std::endl;
            }
            
            // Step 2: Create lazy CNF for MiniSAT (use regular CNFConstructor)
            auto cnf_build_start = std::chrono::high_resolution_clock::now();
            auto cnf_constructor = SATSolverManager::create_cnf_constructor(mdds, true); // true = lazy encoding
            
            // Build the CNF
            CNF cnf = cnf_constructor.construct_cnf();
            
            auto cnf_build_end = std::chrono::high_resolution_clock::now();
            double cnf_build_time = std::chrono::duration<double>(cnf_build_end - cnf_build_start).count();
            total_cnf_build_time += cnf_build_time;
            if (timestep_increase == 0) {
                cnf_vars_start = cnf.count_variables();
                cnf_clauses_start = cnf.count_clauses();
            }
            
            std::cout << "CNFConstructor created successfully" << std::endl;
            std::cout << "Number of variables: " << cnf.count_variables() << std::endl;
            std::cout << "Number of clauses: " << cnf.count_clauses() << std::endl;
            
            // Inner loop: solve and add collision clauses until UNSAT or solution found
            int iteration_count = 0;
            const int max_inner_iterations = 50; // Prevent infinite loops
            double timestep_solver_time = 0.0;
            int timestep_collisions = 0;
            
            while (iteration_count < max_inner_iterations) {
                auto solver_start = std::chrono::high_resolution_clock::now();
                std::cout << "\n--- Inner Iteration " << (iteration_count + 1) << " ---" << std::endl;
                
                // Step 3: Solve with MiniSAT
                std::cout << "=== Solving with MiniSAT ===" << std::endl;
                
                auto result = SATSolverManager::solve_cnf_with_minisat(cnf);
                
                auto solver_end = std::chrono::high_resolution_clock::now();
                double solver_time = std::chrono::duration<double>(solver_end - solver_start).count();
                solver_times_per_iter.push_back(solver_time);
                decisions_per_iter.push_back(result.num_decisions);
                propagations_per_iter.push_back(result.num_propagations);
                tries_per_iter.push_back(1); // Only one try per call in this test
                total_solver_time += solver_time;
                timestep_solver_time += solver_time;
                
                // Log this collision iteration
                SATSolverManager::log_collision_iteration_minisat(
                    "data/solver_log_collisions_minisat.csv",
                    map_name_logged,
                    num_agents_logged,
                    solver_used,
                    current_max_timesteps,
                    iteration_count,
                    cnf_constructor.get_cnf().count_variables(),
                    cnf_constructor.get_cnf().count_clauses(),
                    solver_time,
                    result.num_decisions, // Use decisions for MiniSAT
                    result.num_propagations, // Use propagations for MiniSAT
                    0, // collisions_added (update if you track this)
                    result.satisfiable ? "SAT" : "UNSAT",
                    seed
                );
                
                // Step 4: Process results
                std::cout << "\n=== Results ===" << std::endl;
                if (result.satisfiable) {
                    status = "SAT";
                    std::cout << "SATISFIABLE!" << std::endl;
                    std::cout << "Solve time: " << result.solve_time << "s" << std::endl;
                    std::cout << "Number of decisions: " << result.num_decisions << std::endl;
                    std::cout << "Number of propagations: " << result.num_propagations << std::endl;
                    std::cout << "Total time (including overhead): " << solver_time << "s" << std::endl;
                    
                    // Convert assignment to agent paths
                    auto agent_paths = cnf_constructor.cnf_assignment_to_paths(result.assignment);
                    
                    // Debug: Print the actual extracted paths
                    std::cout << "\n=== DEBUG: Actual Extracted Paths ===" << std::endl;
                    for (const auto& [agent_id, path] : agent_paths) {
                        std::cout << "Agent " << agent_id << " extracted path:" << std::endl;
                        for (size_t t = 0; t < path.size(); ++t) {
                            std::cout << "  t=" << t << ": (" << path[t].first << "," << path[t].second << ")" << std::endl;
                        }
                    }
                    
                    // Print and visualize the paths
                    SATSolverManager::print_agent_paths(agent_paths);
                    
                    // Validate paths
                    std::cout << "\n=== Path Validation ===" << std::endl;
                    bool all_paths_valid = true;
                    for (const auto& [agent_id, path] : agent_paths) {
                        bool valid = cnf_constructor.validate_path(agent_id, path);
                        std::cout << "Agent " << agent_id << " path is " 
                                  << (valid ? "VALID" : "INVALID") << std::endl;
                        if (!valid) all_paths_valid = false;
                    }
                    
                    if (!all_paths_valid) {
                        std::cout << "ERROR: Some paths are invalid!" << std::endl;
                        return 1;
                    }
                    
                    // Test collision detection
                    std::cout << "\n=== Collision Detection ===" << std::endl;
                    
                    // Test vertex collisions
                    auto vertex_collisions = SATSolverManager::find_vertex_collisions(agent_paths);
                    if (vertex_collisions.empty()) {
                        std::cout << "No vertex collisions detected" << std::endl;
                    } else {
                        std::cout << "Vertex collisions detected:" << std::endl;
                        for (const auto& collision : vertex_collisions) {
                            int agent1, agent2, timestep;
                            std::pair<int, int> position;
                            std::tie(agent1, agent2, position, timestep) = collision;
                            std::cout << "  Agents " << agent1 << " and " << agent2 
                                      << " at position (" << position.first << "," << position.second 
                                      << ") at timestep " << timestep << std::endl;
                        }
                    }
                    
                    // Test edge collisions
                    auto edge_collisions = SATSolverManager::find_edge_collisions(agent_paths);
                    if (edge_collisions.empty()) {
                        std::cout << "No edge collisions detected" << std::endl;
                    } else {
                        std::cout << "Edge collisions detected:" << std::endl;
                        for (const auto& collision : edge_collisions) {
                            int agent1, agent2, timestep;
                            std::pair<int, int> pos1, pos2;
                            std::tie(agent1, agent2, pos1, pos2, timestep) = collision;
                            std::cout << "  Agents " << agent1 << " and " << agent2 
                                      << " swap positions between timesteps " << timestep 
                                      << " and " << (timestep + 1) << std::endl;
                            std::cout << "    Agent " << agent1 << ": (" << pos1.first 
                                      << "," << pos1.second << ") -> (" << pos2.first 
                                      << "," << pos2.second << ")" << std::endl;
                            std::cout << "    Agent " << agent2 << ": (" << pos2.first 
                                      << "," << pos2.second << ") -> (" << pos1.first 
                                      << "," << pos1.second << ")" << std::endl;
                        }
                    }
                    
                    // Check if we have a collision-free solution
                    if (vertex_collisions.empty() && edge_collisions.empty()) {
                        std::cout << "\n" << std::string(60, '=') << std::endl;
                        std::cout << "=== SUCCESS: COLLISION-FREE SOLUTION FOUND! ===" << std::endl;
                        std::cout << "=== Final Solution with " << current_max_timesteps << " timesteps ===" << std::endl;
                        std::cout << std::string(60, '=') << std::endl;
                        
                        SATSolverManager::print_agent_paths(agent_paths);
                        
                        // Log the final timestep iteration before exiting
                        cnf_vars_end = cnf.count_variables();
                        cnf_clauses_end = cnf.count_clauses();
                        auto timestep_end = std::chrono::high_resolution_clock::now();
                        double timestep_total_time = std::chrono::duration<double>(timestep_end - timestep_start).count();
                        
                        // Calculate total decisions and propagations for this timestep
                        int total_decisions = 0, total_propagations = 0;
                        for (size_t i = 0; i < decisions_per_iter.size(); ++i) {
                            total_decisions += decisions_per_iter[i];
                            total_propagations += propagations_per_iter[i];
                        }
                        
                        SATSolverManager::log_timestep_iteration_minisat(
                            "data/solver_log_timesteps_minisat.csv",
                            map_name_logged,
                            num_agents_logged,
                            solver_used,
                            current_max_timesteps,
                            cnf_vars_end,
                            cnf_clauses_end,
                            cnf_build_time,
                            timestep_solver_time,
                            timestep_collisions,
                            total_decisions,
                            total_propagations,
                            status,
                            seed
                        );
                        
                        // Log the run summary before exiting
                        static auto main_start = std::chrono::high_resolution_clock::now();
                        auto main_end = std::chrono::high_resolution_clock::now();
                        double total_time = std::chrono::duration<double>(main_end - main_start).count();
                        SATSolverManager::log_run_summary_minisat(
                            "data/solver_log_minisat.csv",
                            map_name_logged,
                            num_agents_logged,
                            solver_used,
                            cnf_vars_start,
                            cnf_clauses_start,
                            cnf_vars_end,
                            cnf_clauses_end,
                            total_time,
                            total_cnf_build_time,
                            total_solver_time,
                            solver_times_per_iter,
                            decisions_per_iter, // Use decisions for MiniSAT
                            propagations_per_iter, // Use propagations for MiniSAT
                            collisions_per_iter,
                            status,
                            seed
                        );
                        
                        std::cout << "\n=== Final Statistics ===" << std::endl;
                        std::cout << "Total timesteps used: " << current_max_timesteps << std::endl;
                        std::cout << "Total iterations: " << (timestep_increase + 1) << std::endl;
                                                std::cout << "Final CNF size: " << cnf.count_variables()
                                  << " variables, " << cnf.count_clauses() << " clauses" << std::endl;
                        
                        std::cout << "\n=== Test completed successfully ===" << std::endl;
                        return 0;
                    }
                    
                    // Add collision prevention clauses and continue
                    std::cout << "\n=== Adding Collision Prevention Clauses ===" << std::endl;
                    
                    // Add collision prevention clauses using CNFConstructor methods
                    int edge_clauses_added = 0;
                    int vertex_clauses_added = 0;
                    
                    for (const auto& collision : vertex_collisions) {
                        int agent1, agent2, timestep;
                        std::pair<int, int> position;
                        std::tie(agent1, agent2, position, timestep) = collision;
                        
                        auto clause = cnf_constructor.add_single_collision_clause(agent1, agent2, position, timestep, true);
                        if (!clause.empty()) {
                            vertex_clauses_added++;
                        }
                    }
                    
                    // Debug: Print all variables in the CNFConstructor
                    std::cout << "\n=== DEBUG: All variables in CNFConstructor ===" << std::endl;
                    const auto& var_map = cnf_constructor.get_variable_map();
                    for (const auto& [key, var_id] : var_map) {
                        int agent_id = std::get<0>(key);
                        auto pos = std::get<1>(key);
                        int t = std::get<2>(key);
                        std::cout << "  Var " << var_id << ": agent " << agent_id << ", pos (" << pos.first << "," << pos.second << "), t=" << t << std::endl;
                    }
                    
                    for (const auto& collision : edge_collisions) {
                        int agent1, agent2, timestep;
                        std::pair<int, int> pos1, pos2;
                        std::tie(agent1, agent2, pos1, pos2, timestep) = collision;
                        
                        std::cout << "\n=== DEBUG: Processing edge collision ===" << std::endl;
                        std::cout << "  Agent1=" << agent1 << ", Agent2=" << agent2 << std::endl;
                        std::cout << "  Pos1=(" << pos1.first << "," << pos1.second << "), Pos2=(" << pos2.first << "," << pos2.second << ")" << std::endl;
                        std::cout << "  Timestep=" << timestep << std::endl;
                        
                        // Check what variables should exist
                        int var1 = cnf_constructor.get_variable_id(agent1, pos1, timestep);
                        int var2 = cnf_constructor.get_variable_id(agent1, pos2, timestep + 1);
                        int var3 = cnf_constructor.get_variable_id(agent2, pos2, timestep);
                        int var4 = cnf_constructor.get_variable_id(agent2, pos1, timestep + 1);
                        
                        std::cout << "  Expected variables: var1=" << var1 << ", var2=" << var2 << ", var3=" << var3 << ", var4=" << var4 << std::endl;
                        
                        auto clause = cnf_constructor.add_single_edge_collision_clause(agent1, agent2, pos1, pos2, timestep, true);
                        if (!clause.empty()) {
                            edge_clauses_added++;
                        }
                    }
                    
                    std::cout << "Added " << edge_clauses_added << " edge collision prevention clauses" << std::endl;
                    std::cout << "Added " << vertex_clauses_added << " vertex collision prevention clauses" << std::endl;
                    
                    if (edge_clauses_added == 0 && vertex_clauses_added == 0) {
                        std::cout << "WARNING: No collision clauses were added despite detecting collisions!" << std::endl;
                        break;
                    }
                    
                    // Rebuild the CNF with the new clauses
                    cnf = cnf_constructor.get_cnf();
                    
                    std::cout << "Updated CNF: " << cnf.count_variables() 
                              << " variables, " << cnf.count_clauses() << " clauses" << std::endl;
                    
                } else {
                    std::cout << "UNSATISFIABLE or UNKNOWN" << std::endl;
                    if (!result.error_message.empty()) {
                        std::cout << "Error: " << result.error_message << std::endl;
                    }
                    std::cout << "No solution found with " << current_max_timesteps << " timesteps" << std::endl;
                    break; // Exit inner loop, increase timesteps
                }
                
                iteration_count++;
                timestep_collisions++;
            }
            cnf_vars_end = cnf.count_variables();
            cnf_clauses_end = cnf.count_clauses();
            auto timestep_end = std::chrono::high_resolution_clock::now();
            double timestep_total_time = std::chrono::duration<double>(timestep_end - timestep_start).count();
            
            // Calculate total decisions and propagations for this timestep
            int total_decisions = 0, total_propagations = 0;
            for (size_t i = 0; i < decisions_per_iter.size(); ++i) {
                total_decisions += decisions_per_iter[i];
                total_propagations += propagations_per_iter[i];
            }
            
            // Log this timestep iteration
            SATSolverManager::log_timestep_iteration_minisat(
                "data/solver_log_timesteps_minisat.csv",
                map_name_logged,
                num_agents_logged,
                solver_used,
                current_max_timesteps,
                cnf_vars_end,
                cnf_clauses_end,
                cnf_build_time,
                timestep_solver_time,
                timestep_collisions,
                total_decisions,
                total_propagations,
                status,
                seed
            );
            
            if (iteration_count >= max_inner_iterations) {
                std::cout << "WARNING: Reached maximum inner iterations (" << max_inner_iterations << ")" << std::endl;
            }
            
            // Increase timesteps for next iteration
            current_max_timesteps++;
            std::cout << "\n=== Increasing timesteps to " << current_max_timesteps << " ===" << std::endl;
        }
        
        // At the end, log the run summary
        // Measure total time from the start of main
        static auto main_start = std::chrono::high_resolution_clock::now();
        auto main_end = std::chrono::high_resolution_clock::now();
        double total_time = std::chrono::duration<double>(main_end - main_start).count();
        SATSolverManager::log_run_summary_minisat(
            "data/solver_log_minisat.csv",
            map_name_logged,
            num_agents_logged,
            solver_used,
            cnf_vars_start,
            cnf_clauses_start,
            cnf_vars_end,
            cnf_clauses_end,
            total_time,
            total_cnf_build_time,
            total_solver_time,
            solver_times_per_iter,
            decisions_per_iter, // Use decisions for MiniSAT
            propagations_per_iter, // Use propagations for MiniSAT
            collisions_per_iter,
            status,
            seed
        );
        
        std::cout << "\n" << std::string(60, '=') << std::endl;
        // Print total time at the end
        std::cout << "Total time (including overhead): " << total_time << "s" << std::endl;
        std::cout << "=== FAILURE: NO SOLUTION FOUND WITHIN " << (initial_max_timesteps + max_timestep_increase) << " TIMESTEPS ===" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 