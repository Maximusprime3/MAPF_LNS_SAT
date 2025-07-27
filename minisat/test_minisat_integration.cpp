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
        std::cout << "=== MiniSAT Integration Test ===" << std::endl;
        
        // Create a simple 2x2 grid
        std::vector<std::vector<char>> grid = {
            {'.', '.'},
            {'.', '.'}
        };
        
        // Start at (0,0), goal at (1,1)
        MDDNode::Position start = {0, 0};
        MDDNode::Position goal = {1, 1};
        
        // Construct MDD with 3 timesteps
        MDDConstructor constructor(grid, start, goal, 3);
        std::shared_ptr<MDD> mdd = constructor.construct_mdd();
        
        std::cout << "MDD constructed successfully" << std::endl;
        std::cout << "MDD: " << mdd->to_string() << std::endl;
        
        // Create CNF from MDD
        std::unordered_map<int, std::shared_ptr<MDD>> mdds;
        mdds[0] = mdd;
        
        CNFConstructor cnf_constructor(mdds, false); // false = eager encoding
        CNF cnf = cnf_constructor.construct_cnf();
        
        std::cout << "CNF constructed successfully" << std::endl;
        std::cout << "Number of variables: " << cnf.count_variables() << std::endl;
        std::cout << "Number of clauses: " << cnf.count_clauses() << std::endl;
        
        // Test MiniSAT solving
        std::cout << "\n=== Testing MiniSAT ===" << std::endl;
        auto minisat_result = SATSolverManager::solve_cnf_with_minisat(cnf);
        
        if (minisat_result.satisfiable) {
            std::cout << "MiniSAT: SATISFIABLE!" << std::endl;
            std::cout << "Solve time: " << minisat_result.solve_time << "s" << std::endl;
            std::cout << "Number of decisions: " << minisat_result.num_decisions << std::endl;
            std::cout << "Assignment (first 10 variables): ";
            for (int i = 0; i < std::min(10, (int)minisat_result.assignment.size()); ++i) {
                std::cout << minisat_result.assignment[i] << " ";
            }
            std::cout << std::endl;
            
            // Extract and validate paths
            auto agent_paths = cnf_constructor.cnf_assignment_to_paths(minisat_result.assignment);
            std::cout << "\nAgent paths:" << std::endl;
            for (const auto& [agent_id, path] : agent_paths) {
                std::cout << "Agent " << agent_id << ": ";
                for (const auto& pos : path) {
                    std::cout << "(" << pos.first << "," << pos.second << ") ";
                }
                std::cout << std::endl;
            }
        } else {
            std::cout << "MiniSAT: UNSATISFIABLE" << std::endl;
            if (!minisat_result.error_message.empty()) {
                std::cout << "Error: " << minisat_result.error_message << std::endl;
            }
        }
        
        // Compare with ProbSAT
        std::cout << "\n=== Testing ProbSAT for comparison ===" << std::endl;
        auto probsat_result = SATSolverManager::solve_cnf_with_probsat(cnf, 42, 1, 10000);
        
        if (probsat_result.satisfiable) {
            std::cout << "ProbSAT: SATISFIABLE!" << std::endl;
            std::cout << "Solve time: " << probsat_result.solve_time << "s" << std::endl;
            std::cout << "Number of flips: " << probsat_result.num_flips << std::endl;
        } else {
            std::cout << "ProbSAT: UNSATISFIABLE" << std::endl;
            if (!probsat_result.error_message.empty()) {
                std::cout << "Error: " << probsat_result.error_message << std::endl;
            }
        }
        
        std::cout << "\n=== Test completed successfully ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 