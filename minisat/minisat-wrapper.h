#ifndef MINISAT_WRAPPER_H
#define MINISAT_WRAPPER_H

#include <vector>
#include <string>
#include <chrono>
#include <cstddef>
#include <unordered_set>

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

    // Incremental solving interface. Subsequent calls add only new clauses and
    // reuse the solver state so learnt clauses and polarities are preserved.
    MiniSatSolution solve_cnf_incremental(const std::vector<std::vector<int>>& clauses,
                        const std::vector<int>* initial_assignment = nullptr);

    // Resets the incremental session (clears loaded clauses and solver state).
    void reset_incremental();

    // Alternative interface that takes CNF in the same format as ProbSAT
    MiniSatSolution solve_cnf_with_pointers(int numVars, 
                                           int numClauses, 
                                           int** clauses,
                                           const std::vector<int>* initial_assignment = nullptr);

private:
    Minisat::Solver* solver;
    std::size_t clauses_loaded;
    bool incremental_mode;

    MiniSatSolution solve_with_clause_range(const std::vector<std::vector<int>>& clauses,
                                            const std::vector<int>* initial_assignment,
                                            std::size_t start_index,
                                            bool incremental_run);
    
    // Helper functions
    void reset_solver();
    bool add_clause_to_solver(const std::vector<int>& clause);
    std::vector<int> extract_assignment(int num_vars);
};

#endif // MINISAT_WRAPPER_H 