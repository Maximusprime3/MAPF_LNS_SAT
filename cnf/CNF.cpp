#include "CNF.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#ifdef __GNUC__
#include <execinfo.h>
#endif

CNF::CNF() {
    // Constructor initializes empty CNF
}

void CNF::add_clause(const std::vector<int>& clause) {
    if (clause.empty()) {
        std::cerr << "[CNF] WARNING: Attempted to add empty clause!" << std::endl;
#ifdef __GNUC__
        void* callstack[10];
        int frames = backtrace(callstack, 10);
        char** strs = backtrace_symbols(callstack, frames);
        for (int i = 0; i < frames; ++i) {
            std::cerr << strs[i] << std::endl;
        }
        free(strs);
#endif
    }
    clauses.push_back(clause);
    
    // Update variables set
    for (int literal : clause) {
        variables.insert(std::abs(literal));
    }
}

int CNF::count_variables() const {
    return variables.size();
}

int CNF::count_clauses() const {
    return clauses.size();
}

std::string CNF::to_dimacs() const {
    std::stringstream ss;
    
    // Add metadata line
    ss << get_metadata();
    
    // Add each clause
    for (const auto& clause : clauses) {
        for (int literal : clause) {
            ss << literal << " ";
        }
        ss << "0\n"; // DIMACS format requires 0 at end of each clause
    }
    
    return ss.str();
}

std::string CNF::get_metadata() const {
    std::stringstream ss;
    ss << "p cnf " << count_variables() << " " << count_clauses() << "\n";
    return ss.str();
}

const std::vector<std::vector<int>>& CNF::get_clauses() const {
    return clauses;
}

void CNF::clear() {
    clauses.clear();
    variables.clear();
}

void cnf_to_probsat_format(const CNF& cnf, std::vector<int*>& clause_pointers, std::vector<std::vector<int>>& clause_storage) {
    int num_vars = cnf.count_variables();
    int clause_idx = 1;
    const auto& all_clauses = cnf.get_clauses();
    int total_clauses = all_clauses.size();
    clause_storage.reserve(total_clauses);
    
    // First, build all clause vectors
    for (const auto& clause : all_clauses) {
        bool is_zero_clause = clause.empty() ||
                              (clause.size() == 1 && clause[0] == 0) ||
                              std::all_of(clause.begin(), clause.end(), [](int lit){ return lit == 0; });
        if (is_zero_clause) {
            std::cerr << "[CNF->ProbSAT] Skipping zero/empty clause at index " << clause_idx << std::endl;
            clause_idx++;
            continue;
        }
        bool valid = true;
        for (int lit : clause) {
            if (lit == 0 || std::abs(lit) > num_vars) {
                std::cerr << "[CNF->ProbSAT] Invalid literal " << lit << " in clause " << clause_idx << ", skipping this clause!" << std::endl;
                valid = false;
                break;
            }
        }
        if (!valid) { 
            clause_idx++; 
            continue; 
        }
        std::vector<int> clause_copy = clause;
        clause_copy.push_back(0); // null terminator
        clause_storage.push_back(clause_copy);
        clause_idx++;
    }
    
    // Now, build the pointer array
    clause_pointers.clear();
    clause_pointers.push_back(nullptr); // dummy for 1-based indexing
    for (auto& clause_vec : clause_storage) {
        clause_pointers.push_back(clause_vec.data());
    }
    std::cerr << "[CNF->ProbSAT] Total clause pointers: " << clause_pointers.size()-1 << ", total clauses: " << total_clauses << std::endl;
} 