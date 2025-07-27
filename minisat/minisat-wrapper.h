#ifndef MINISAT_WRAPPER_H
#define MINISAT_WRAPPER_H

#include <vector>
#include <string>
#include <chrono>

// Forward declaration to avoid including MiniSAT headers here
namespace Minisat {
    class Solver;
}

// Forward declaration of the result structure
struct MiniSatSolution;

// MiniSAT wrapper class
class MiniSatWrapper {
public:
    MiniSatWrapper();
    ~MiniSatWrapper();
    
    // Main solving function
    MiniSatSolution solve_cnf(const std::vector<std::vector<int>>& clauses, 
                             const std::vector<int>* initial_assignment = nullptr);
    
    // Alternative interface that takes CNF in the same format as ProbSAT
    MiniSatSolution solve_cnf_with_pointers(int numVars, 
                                           int numClauses, 
                                           int** clauses,
                                           const std::vector<int>* initial_assignment = nullptr);

private:
    Minisat::Solver* solver;
    
    // Helper functions
    void reset_solver();
    bool add_clause_to_solver(const std::vector<int>& clause);
    std::vector<int> extract_assignment(int num_vars);
};

#endif // MINISAT_WRAPPER_H 