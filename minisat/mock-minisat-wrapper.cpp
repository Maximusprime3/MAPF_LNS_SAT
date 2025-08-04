#include "mock-minisat-wrapper.h"
#include "../SATSolverManager.h" // For MiniSatSolution definition
#include <random>
#include <algorithm>
#include <thread>

MockMiniSatWrapper::MockMiniSatWrapper() : solver_initialized(false) {
    reset_solver();
}

MockMiniSatWrapper::~MockMiniSatWrapper() {
    // Nothing to clean up for mock
}

void MockMiniSatWrapper::reset_solver() {
    solver_initialized = true;
}

MiniSatSolution MockMiniSatWrapper::solve_cnf(const std::vector<std::vector<int>>& clauses, 
                                             const std::vector<int>* initial_assignment) {
    MiniSatSolution result;
    result.satisfiable = false;
    result.num_decisions = 0;
    result.solve_time = 0.0;
    result.error_message = "";
    
    try {
        reset_solver();
        
        // Add clauses to the solver
        for (const auto& clause : clauses) {
            if (!add_clause_to_solver(clause)) {
                result.error_message = "Failed to add clause to mock MiniSAT solver";
                return result;
            }
        }
        
        // Mock solving
        auto start_time = std::chrono::high_resolution_clock::now();
        
        bool satisfiable = mock_solve();
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        result.satisfiable = satisfiable;
        result.solve_time = duration.count() / 1000.0; // Convert to seconds
        result.num_decisions = 42; // Mock value
        
        if (satisfiable) {
            // Generate a mock assignment based on the actual number of variables
            int num_vars = 0;
            for (const auto& clause : clauses) {
                for (int literal : clause) {
                    if (literal != 0) {
                        num_vars = std::max(num_vars, abs(literal));
                    }
                }
            }
            result.assignment = extract_assignment(num_vars);
        }
        
    } catch (const std::exception& e) {
        result.error_message = std::string("Exception during mock MiniSAT solving: ") + e.what();
    }
    
    return result;
}

MiniSatSolution MockMiniSatWrapper::solve_cnf_with_pointers(int numVars, 
                                                           int numClauses, 
                                                           int** clauses,
                                                           const std::vector<int>* initial_assignment) {
    // Convert pointer format to vector format
    std::vector<std::vector<int>> clauses_vec;
    
    for (int i = 0; i < numClauses; ++i) {
        std::vector<int> clause;
        int* clause_ptr = clauses[i];
        
        // Read until we hit 0 (null terminator)
        int j = 0;
        while (clause_ptr[j] != 0) {
            clause.push_back(clause_ptr[j]);
            j++;
        }
        
        if (!clause.empty()) {
            clauses_vec.push_back(clause);
        }
    }
    
    return solve_cnf(clauses_vec, initial_assignment);
}

bool MockMiniSatWrapper::add_clause_to_solver(const std::vector<int>& clause) {
    if (clause.empty()) {
        return true; // Mock success
    }
    
    // Mock: just check if clause is valid
    for (int literal : clause) {
        if (literal == 0) continue; // Skip null terminators
        if (abs(literal) < 1) {
            return false; // Invalid variable
        }
    }
    
    return true;
}

std::vector<int> MockMiniSatWrapper::extract_assignment(int num_vars) {
    std::vector<int> assignment(num_vars);
    
    // Generate a more realistic mock assignment
    // Use a pattern that's more likely to produce valid paths
    for (int i = 0; i < num_vars; ++i) {
        // Use a pattern that favors some variables being true
        // This should help create more realistic path assignments
        if (i % 3 == 0 || i % 5 == 0) {
            assignment[i] = 1;  // Make some variables true
        } else {
            assignment[i] = 0;  // Make others false
        }
    }
    
    return assignment;
}

bool MockMiniSatWrapper::mock_solve() {
    // Mock: simulate some solving time and return SAT
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return true; // Always return SAT for mock
} 