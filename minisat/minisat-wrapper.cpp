#include "minisat-wrapper.h"
#include "SATSolverManager.h" // For MiniSatSolution definition

// Include MiniSAT headers
#include "minisat-master/minisat/core/Solver.h"
#include "minisat-master/minisat/core/SolverTypes.h"
#include "minisat-master/minisat/mtl/Vec.h"
#include <unordered_set>
#include <set>
#include <algorithm>
#include <sstream>

using namespace Minisat;

MiniSatWrapper::MiniSatWrapper() : solver(nullptr), clauses_loaded(0), incremental_mode(false) {
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
    clauses_loaded = 0;
    incremental_mode = false;
}

MiniSatSolution MiniSatWrapper::solve_cnf(const std::vector<std::vector<int>>& clauses,
                                          const std::vector<int>* initial_assignment) {
    incremental_mode = false;
    clauses_loaded = 0;
    return solve_with_clause_range(clauses, initial_assignment, 0, false, false);
}

MiniSatSolution MiniSatWrapper::solve_cnf_incremental(const std::vector<std::vector<int>>& clauses,
                                                    const std::vector<int>* initial_assignment,
                                                    bool use_assumptions) {
    if (!incremental_mode) {
        reset_solver();
        incremental_mode = true;
        clauses_loaded = 0;
    }

    if (clauses_loaded > clauses.size()) {
        // CNF was rebuilt; restart incremental session from scratch.
        reset_solver();
        incremental_mode = true;
    }

    std::size_t start_index = clauses_loaded;
    auto result = solve_with_clause_range(clauses, initial_assignment, start_index, true, use_assumptions);
    clauses_loaded = clauses.size();
    return result;
}

void MiniSatWrapper::reset_incremental() {
    reset_solver();
}


MiniSatSolution MiniSatWrapper::solve_with_clause_range(const std::vector<std::vector<int>>& clauses,
                                                        const std::vector<int>* initial_assignment,
                                                        std::size_t start_index,
                                                        bool incremental_run,
                                                        bool use_assumptions) {
    MiniSatSolution result;
    result.satisfiable = false;
    result.num_decisions = 0;
    result.num_propagations = 0;
    result.solve_time = 0.0;
    result.error_message = "";
    
    try {
        if (!solver) {
            reset_solver();
        }
        
        // Add clauses to the solver with diagnostics
        std::cout << "DEBUG: Adding "
                  << (clauses.size() - start_index)
                  << " new clauses to MiniSAT"
                  << (incremental_run ? " (incremental run)" : "")
                  << std::endl;

        auto normalize_clause = [](const std::vector<int>& raw,
                                   bool& tautology,
                                   size_t& simplified_lits) -> std::vector<int> {
            tautology = false;
            std::set<int> litset;
            for (int lit : raw) {
                if (lit == 0) continue;
                if (litset.count(-lit)) { tautology = true; return {}; }
                litset.insert(lit);
            }
            std::vector<int> norm(litset.begin(), litset.end());
            simplified_lits += (raw.size() > norm.size() ? raw.size() - norm.size() : 0);
            return norm;
        };

        std::unordered_set<std::string> seen;
        size_t num_added = 0;
        size_t num_tautologies = 0;
        size_t num_duplicates = 0;
        size_t num_units = 0;
        size_t num_empties = 0;
        size_t num_simplified_literals = 0;
        size_t num_added_but_no_db_increase = 0; // clauses accepted but didn't increase nClauses()

        for (size_t i = start_index; i < clauses.size(); ++i) {
            const auto& clause = clauses[i];

            bool is_tauto = false;
            auto norm = normalize_clause(clause, is_tauto, num_simplified_literals);
            if (is_tauto) { num_tautologies++; continue; }

            // Create a stable string key for duplicate detection
            std::ostringstream oss;
            for (size_t k = 0; k < norm.size(); ++k) {
                if (k) oss << ',';
                oss << norm[k];
            }
            std::string key = oss.str();
            if (!seen.insert(key).second) { num_duplicates++; continue; }

            if (norm.empty()) { num_empties++; }
            if (norm.size() == 1) { num_units++; }

            int before_clauses = solver->nClauses();
            if (!add_clause_to_solver(norm)) {
                // MiniSAT's addClause API returns false when the accumulated
                // formula becomes contradictory. That is a normal UNSAT
                // result, not a wrapper/backend failure; leave error_message
                // empty so higher solver layers classify it as exhaustion.
                result.satisfiable = false;
                return result;
            }
            int after_clauses = solver->nClauses();
            if (norm.size() > 1 && after_clauses == before_clauses) {
                // Clause was accepted but database size did not grow (likely satisfied/subsumed by propagation)
                num_added_but_no_db_increase++;
            }
            num_added++;
        }
        std::cout << "DEBUG: Clause load summary: input=" << (clauses.size() - start_index)
                  << ", added=" << num_added
                  << ", tautologies_skipped=" << num_tautologies
                  << ", duplicates_skipped=" << num_duplicates
                  << ", unit_clauses=" << num_units
                  << ", empties=" << num_empties
                  << ", intra_clause_literals_removed=" << num_simplified_literals
                  << ", added_but_db_unchanged=" << num_added_but_no_db_increase
                  << std::endl;
        std::cout << "DEBUG: After adding clauses, solver has " << solver->nClauses() << " clauses and " << solver->nVars() << " variables" << std::endl;

        vec<Lit> assumption_lits;
        // Set initial assignment if provided
        if (initial_assignment != nullptr) {
            // MiniSAT uses 0-based indexing, so we need to convert
            for (size_t i = 0; i < initial_assignment->size(); ++i) {
                int var_id = static_cast<int>(i); // 0-based
                if (var_id >= solver->nVars()) {
                    std::cout << "DEBUG: ERROR: Initial assignment variable " << var_id << " is out of bounds" << std::endl;
                    continue;
                }
                int value = (*initial_assignment)[i];
                
                if (use_assumptions) {
                    if (value == 1) {
                        assumption_lits.push(mkLit(var_id, /*sign=*/false));
                    } else if (value == 0) {
                        assumption_lits.push(mkLit(var_id, /*sign=*/true));
                    }
                } else {
                    // Set the polarity based on the initial assignment
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
        
        bool satisfiable = false;
        if (use_assumptions && initial_assignment != nullptr) {
            satisfiable = solver->solve(assumption_lits);
        } else {
            satisfiable = solver->solve();
        }
        
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);        
        result.satisfiable = satisfiable;
        result.solve_time = duration.count();
        result.num_decisions = solver->decisions;
        result.num_propagations = solver->propagations;
        
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
