#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <sstream>

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
    try {
        std::cout << "=== MAPF LNS SAT with SATSolverManager - Iterative Loop Test ===" << std::endl;
        
        // Configuration
        const std::string map_path = "mapf-map/empty-16-16.map";
        const std::string scenario_path = "mapf-scen-even/scen-even/empty-16-16-even-1.scen";
        const int num_agents = 3;  // Increased to 3 agents
        const int initial_max_timesteps = 2;  // Start with 2 timesteps
        const int max_timestep_increase = 10;  // Maximum additional timesteps
        const long long seed = 42;
        const long long max_runs = 1;
        const long long max_flips = 10000;
        
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
        
        // Main iterative loop
        for (int timestep_increase = 0; timestep_increase <= max_timestep_increase; ++timestep_increase) {
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
            
            // Step 2: Create lazy CNF
            std::cout << "\n=== Creating Lazy CNFProbSATConstructor ===" << std::endl;
            auto cnf_constructor = SATSolverManager::create_cnf_probsat_constructor(mdds, true); // true = lazy encoding
            
            std::cout << "CNFProbSATConstructor created successfully" << std::endl;
            std::cout << "Number of variables: " << cnf_constructor->get_probsat_num_variables() << std::endl;
            std::cout << "Number of clauses: " << cnf_constructor->get_probsat_num_clauses() << std::endl;
            
            // Inner loop: solve and add collision clauses until UNSAT or solution found
            int iteration_count = 0;
            const int max_inner_iterations = 50; // Prevent infinite loops
            
            while (iteration_count < max_inner_iterations) {
                std::cout << "\n--- Inner Iteration " << (iteration_count + 1) << " ---" << std::endl;
                
                // Step 3: Solve with ProbSAT
                std::cout << "=== Solving with ProbSAT ===" << std::endl;
                
                auto start_time = std::chrono::high_resolution_clock::now();
                
                ProbSATSolution result = SATSolverManager::solve_cnf_with_probsat(
                    cnf_constructor,
                    seed,
                    max_runs,
                    max_flips
                );
                
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                
                // Step 4: Process results
                std::cout << "\n=== Results ===" << std::endl;
                if (result.satisfiable) {
                    std::cout << "SATISFIABLE!" << std::endl;
                    std::cout << "Solve time: " << result.solve_time << "s" << std::endl;
                    std::cout << "Number of flips: " << result.num_flips << std::endl;
                    std::cout << "Total time (including overhead): " << duration.count() << "ms" << std::endl;
                    
                    // Convert assignment to agent paths
                    auto agent_paths = cnf_constructor->cnf_assignment_to_paths(result.assignment);
                    
                    // Print and visualize the paths
                    SATSolverManager::print_agent_paths(agent_paths);
                    
                    // Validate paths
                    std::cout << "\n=== Path Validation ===" << std::endl;
                    bool all_paths_valid = true;
                    for (const auto& [agent_id, path] : agent_paths) {
                        bool valid = cnf_constructor->validate_path(agent_id, path);
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
                        
                        std::cout << "\n=== Final Statistics ===" << std::endl;
                        std::cout << "Total timesteps used: " << current_max_timesteps << std::endl;
                        std::cout << "Total iterations: " << (timestep_increase + 1) << std::endl;
                        std::cout << "Final CNF size: " << cnf_constructor->get_probsat_num_variables() 
                                  << " variables, " << cnf_constructor->get_probsat_num_clauses() << " clauses" << std::endl;
                        
                        std::cout << "\n=== Test completed successfully ===" << std::endl;
                        return 0;
                    }
                    
                    // Add collision prevention clauses and continue
                    std::cout << "\n=== Adding Collision Prevention Clauses ===" << std::endl;
                    
                    int edge_clauses_added = SATSolverManager::add_edge_collision_prevention_clauses(
                        cnf_constructor, edge_collisions);
                    int vertex_clauses_added = SATSolverManager::add_vertex_collision_prevention_clauses(
                        cnf_constructor, vertex_collisions);
                    
                    std::cout << "Added " << edge_clauses_added << " edge collision prevention clauses" << std::endl;
                    std::cout << "Added " << vertex_clauses_added << " vertex collision prevention clauses" << std::endl;
                    
                    if (edge_clauses_added == 0 && vertex_clauses_added == 0) {
                        std::cout << "WARNING: No collision clauses were added despite detecting collisions!" << std::endl;
                        break;
                    }
                    
                    // Re-build the clause pointers for ProbSAT after adding new clauses
                    cnf_constructor->build_clause_pointers();
                    
                    std::cout << "Updated CNF: " << cnf_constructor->get_probsat_num_variables() 
                              << " variables, " << cnf_constructor->get_probsat_num_clauses() << " clauses" << std::endl;
                    
                } else {
                    std::cout << "UNSATISFIABLE or UNKNOWN" << std::endl;
                    if (!result.error_message.empty()) {
                        std::cout << "Error: " << result.error_message << std::endl;
                    }
                    std::cout << "No solution found with " << current_max_timesteps << " timesteps" << std::endl;
                    break; // Exit inner loop, increase timesteps
                }
                
                iteration_count++;
            }
            
            if (iteration_count >= max_inner_iterations) {
                std::cout << "WARNING: Reached maximum inner iterations (" << max_inner_iterations << ")" << std::endl;
            }
            
            // Increase timesteps for next iteration
            current_max_timesteps++;
            std::cout << "\n=== Increasing timesteps to " << current_max_timesteps << " ===" << std::endl;
        }
        
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "=== FAILURE: NO SOLUTION FOUND WITHIN " << (initial_max_timesteps + max_timestep_increase) << " TIMESTEPS ===" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 