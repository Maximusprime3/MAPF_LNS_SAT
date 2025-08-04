#include "minisat-wrapper.h"
#include "SATSolverManager.h" // For MiniSatSolution definition

// Include MiniSAT headers
#include "minisat-master/minisat/core/Solver.h"
#include "minisat-master/minisat/core/SolverTypes.h"
#include "minisat-master/minisat/mtl/Vec.h"

using namespace Minisat;

MiniSatWrapper::MiniSatWrapper() : solver(nullptr) {
    reset_solver();
}

MiniSatWrapper::~MiniSatWrapper() {
    if (solver) {
        delete solver;
        solver = nullptr;
    }
}

void MiniSatWrapper::reset_solver() {
    if (solver) {
        delete solver;
    }
    solver = new Solver();
}

MiniSatSolution MiniSatWrapper::solve_cnf(const std::vector<std::vector<int>>& clauses, 
                                         const std::vector<int>* initial_assignment) {
    MiniSatSolution result;
    result.satisfiable = false;
    result.num_decisions = 0;
    result.solve_time = 0.0;
    result.error_message = "";
    
    try {
        reset_solver();
        
        // Add clauses to the solver
        std::cout << "DEBUG: Adding " << clauses.size() << " clauses to MiniSAT" << std::endl;
        for (size_t i = 0; i < clauses.size(); ++i) {
            const auto& clause = clauses[i];
            std::cout << "DEBUG: Adding clause " << i << ": ";
            for (int lit : clause) std::cout << lit << " ";
            std::cout << std::endl;
            
            if (!add_clause_to_solver(clause)) {
                result.error_message = "Failed to add clause to MiniSAT solver";
                return result;
            }
        }
        std::cout << "DEBUG: After adding clauses, solver has " << solver->nVars() << " variables" << std::endl;
        
        // Set initial assignment if provided
        if (initial_assignment != nullptr) {
            // MiniSAT uses 0-based indexing, so we need to convert
            for (size_t i = 0; i < initial_assignment->size(); ++i) {
                int var_id = i; // 0-based
                int value = (*initial_assignment)[i];
                
                if (var_id < solver->nVars()) {
                    // Set the polarity based on the initial assignment
                    // Note: MiniSAT doesn't have a direct way to set initial assignments
                    // We'll use setPolarity as a heuristic
                    if (value == 1) {
                        solver->setPolarity(var_id, l_True);
                    } else if (value == 0) {
                        solver->setPolarity(var_id, l_False);
                    }
                }
            }
        }
        
        // Solve the problem
        auto start_time = std::chrono::high_resolution_clock::now();
        
        bool satisfiable = solver->solve();
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        result.satisfiable = satisfiable;
        result.solve_time = duration.count() / 1000.0; // Convert to seconds
        result.num_decisions = solver->decisions;
        
        if (satisfiable) {
            // Extract the assignment
            result.assignment = extract_assignment(solver->nVars());
            std::cout << "DEBUG: MiniSAT found " << solver->nVars() << " variables" << std::endl;
            std::cout << "DEBUG: Assignment size: " << result.assignment.size() << std::endl;
            std::cout << "DEBUG: First few assignment values: ";
            for (size_t i = 0; i < std::min(result.assignment.size(), size_t(10)); ++i) {
                std::cout << result.assignment[i] << " ";
            }
            std::cout << std::endl;
        }
        
    } catch (const std::exception& e) {
        result.error_message = std::string("Exception during MiniSAT solving: ") + e.what();
    }
    
    return result;
}

MiniSatSolution MiniSatWrapper::solve_cnf_with_pointers(int numVars, 
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

bool MiniSatWrapper::add_clause_to_solver(const std::vector<int>& clause) {
    if (clause.empty()) {
        return solver->addEmptyClause();
    }
    
    vec<Lit> lits;
    
    for (int literal : clause) {
        if (literal == 0) continue; // Skip null terminators
        
        int var = abs(literal) - 1; // Convert to 0-based indexing
        bool sign = (literal < 0);
        
        // Ensure the variable exists
        while (var >= solver->nVars()) {
            solver->newVar();
        }
        
        lits.push(mkLit(var, sign));
    }
    
    return solver->addClause(lits);
}

std::vector<int> MiniSatWrapper::extract_assignment(int num_vars) {
    // MiniSAT uses 0-based indexing, return assignment as-is
    // The CNFConstructor will handle the conversion to 1-based indexing
    std::vector<int> assignment(num_vars);
    
    for (int i = 0; i < num_vars; ++i) {
        lbool value = solver->modelValue(i);
        if (value == l_True) {
            assignment[i] = 1;
        } else if (value == l_False) {
            assignment[i] = 0;
        } else {
            assignment[i] = 0; // Default to false for undefined
        }
    }
    
    return assignment;
} 