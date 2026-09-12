#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <iomanip>

// Include our MAPF components
#include "mdd/MDDConstructor.h"
#include "mdd/MDD.h"
#include "mdd/MDDNode.h"
#include "cnf/CNFConstructor.h"
#include "cnf/CNF.h"
#include "SATSolverManager.h"

int main() {
    try {
        std::cout << "=== MiniSAT LNS Usage Example ===" << std::endl;
        
        // Configuration
        const std::string map_path = "mapf-map/empty-16-16.map";
        const std::string scenario_path = "mapf-scen-even/scen-even/empty-16-16-even-1.scen";
        const int num_agents = 2;
        const int initial_max_timesteps = 3;
        const int max_timestep_increase = 5;
        
        std::cout << "Loading map and scenario..." << std::endl;
        
        // Load map and scenario (simplified for example)
        auto grid = SATSolverManager::load_map(map_path);
        auto entries = SATSolverManager::create_dataframe_from_file(scenario_path);
        auto starts_and_goals = SATSolverManager::create_starts_and_goals(entries, num_agents);
        
        if (starts_and_goals.empty()) {
            std::cerr << "No start/goal sets found!" << std::endl;
            return 1;
        }
        
        // Use the first start/goal set
        auto [starts, goals] = starts_and_goals[0];
        
        std::cout << "Loaded " << starts.size() << " agents" << std::endl;
        
        // Compute initial max timesteps and distance matrices
        auto [distance_matrices, computed_max_timesteps] = SATSolverManager::compute_max_timesteps(grid, starts, goals);
        int current_max_timesteps = std::max(computed_max_timesteps, initial_max_timesteps);
        
        std::cout << "Starting with " << current_max_timesteps << " timesteps" << std::endl;
        
        // Main LNS loop with MiniSAT
        for (int timestep_increase = 0; timestep_increase <= max_timestep_increase; ++timestep_increase) {
            std::cout << "\n" << std::string(50, '=') << std::endl;
            std::cout << "=== ITERATION " << (timestep_increase + 1) << " (Timesteps: " << current_max_timesteps << ") ===" << std::endl;
            std::cout << std::string(50, '=') << std::endl;
            
            // Step 1: Create MDDs for current timesteps
            std::cout << "Creating MDDs..." << std::endl;
            auto mdds = SATSolverManager::create_mdds(grid, starts, goals, current_max_timesteps, distance_matrices);
            
            // Step 2: Create lazy CNF
            std::cout << "Creating CNF..." << std::endl;
            auto cnf_constructor = SATSolverManager::create_cnf_probsat_constructor(mdds, true); // true = lazy encoding
            
            std::cout << "CNF: " << cnf_constructor->get_probsat_num_variables() 
                      << " variables, " << cnf_constructor->get_probsat_num_clauses() << " clauses" << std::endl;
            
            // Step 3: Solve with MiniSAT
            std::cout << "Solving with MiniSAT..." << std::endl;
            auto minisat_result = SATSolverManager::solve_cnf_with_minisat(cnf_constructor);
            
            if (minisat_result.satisfiable) {
                std::cout << "MiniSAT: SATISFIABLE!" << std::endl;
                std::cout << "Solve time: " << minisat_result.solve_time << "s" << std::endl;
                std::cout << "Number of decisions: " << minisat_result.num_decisions << std::endl;
                
                // Extract agent paths
                auto agent_paths = cnf_constructor->cnf_assignment_to_paths(minisat_result.assignment);
                
                // Check for collisions
                auto [vertex_collisions, edge_collisions] = SATSolverManager::find_all_collisions(agent_paths);
                
                if (vertex_collisions.empty() && edge_collisions.empty()) {
                    std::cout << "\n" << std::string(50, '=') << std::endl;
                    std::cout << "=== SUCCESS: COLLISION-FREE SOLUTION FOUND! ===" << std::endl;
                    std::cout << "=== Using MiniSAT with " << current_max_timesteps << " timesteps ===" << std::endl;
                    std::cout << std::string(50, '=') << std::endl;
                    
                    SATSolverManager::print_agent_paths(agent_paths);
                    
                    std::cout << "\n=== MiniSAT LNS Example completed successfully ===" << std::endl;
                    return 0;
                } else {
                    std::cout << "Collisions detected, adding collision clauses..." << std::endl;
                    
                    // Add collision prevention clauses
                    int edge_clauses_added = SATSolverManager::add_edge_collision_prevention_clauses(
                        cnf_constructor, edge_collisions);
                    int vertex_clauses_added = SATSolverManager::add_vertex_collision_prevention_clauses(
                        cnf_constructor, vertex_collisions);
                    
                    std::cout << "Added " << edge_clauses_added << " edge collision clauses" << std::endl;
                    std::cout << "Added " << vertex_clauses_added << " vertex collision clauses" << std::endl;
                    
                    // Re-build clause pointers for next iteration
                    cnf_constructor->build_clause_pointers();
                    
                    // Continue with next iteration (collision resolution)
                    continue;
                }
            } else {
                std::cout << "MiniSAT: UNSATISFIABLE" << std::endl;
                if (!minisat_result.error_message.empty()) {
                    std::cout << "Error: " << minisat_result.error_message << std::endl;
                }
                std::cout << "No solution found with " << current_max_timesteps << " timesteps" << std::endl;
            }
            
            // Increase timesteps for next iteration
            current_max_timesteps++;
            std::cout << "Increasing timesteps to " << current_max_timesteps << std::endl;
        }
        
        std::cout << "\n" << std::string(50, '=') << std::endl;
        std::cout << "=== FAILURE: NO SOLUTION FOUND WITHIN " << (initial_max_timesteps + max_timestep_increase) << " TIMESTEPS ===" << std::endl;
        std::cout << std::string(50, '=') << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 