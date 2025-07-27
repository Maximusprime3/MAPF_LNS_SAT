#ifndef SIMPLE_MINISAT_WRAPPER_H
#define SIMPLE_MINISAT_WRAPPER_H

#include <vector>
#include <string>
#include <chrono>

// Forward declaration of the result structure
struct MiniSatSolution;

// Simple MiniSAT wrapper class
class SimpleMiniSatWrapper {
public:
    SimpleMiniSatWrapper();
    ~SimpleMiniSatWrapper();
    
    // Main solving function
    MiniSatSolution solve_cnf(const std::vector<std::vector<int>>& clauses, 
                             const std::vector<int>* initial_assignment = nullptr);
    
    // Alternative interface that takes CNF in the same format as ProbSAT
    MiniSatSolution solve_cnf_with_pointers(int numVars, 
                                           int numClauses, 
                                           int** clauses,
                                           const std::vector<int>* initial_assignment = nullptr);

private:
    // Internal solver pointer (opaque)
    void* solver;
    
    // Helper functions
    void reset_solver();
    bool add_clause_to_solver(const std::vector<int>& clause);
    std::vector<int> extract_assignment(int num_vars);
};

#endif // SIMPLE_MINISAT_WRAPPER_H 