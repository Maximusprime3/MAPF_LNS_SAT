#include "CNFProbSAT.h"
#include <iostream>
#include <algorithm>

CNFProbSAT::CNFProbSAT() : probSAT_format_ready(false) {
    // Constructor initializes with ProbSAT format not ready
}

CNFProbSAT::~CNFProbSAT() {
    // Destructor cleans up allocated memory
    clear_probSAT_format();
}

void CNFProbSAT::generate_probSAT_format() {
    // Clear any existing ProbSAT format data
    clear_probSAT_format();
    
    int num_vars = count_variables();
    int clause_idx = 1;
    const auto& all_clauses = get_clauses();
    int total_clauses = all_clauses.size();
    
    clause_storage.reserve(total_clauses);
    
    // First, build all clause vectors with null terminators
    for (const auto& clause : all_clauses) {
        bool is_zero_clause = clause.empty() ||
                              (clause.size() == 1 && clause[0] == 0) ||
                              std::all_of(clause.begin(), clause.end(), [](int lit){ return lit == 0; });
        if (is_zero_clause) {
            std::cerr << "[CNFProbSAT] Skipping zero/empty clause at index " << clause_idx << std::endl;
            clause_idx++;
            continue;
        }
        
        bool valid = true;
        for (int lit : clause) {
            if (lit == 0 || std::abs(lit) > num_vars) {
                std::cerr << "[CNFProbSAT] Invalid literal " << lit << " in clause " << clause_idx << ", skipping this clause!" << std::endl;
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
    
    // Now, build the pointer array with 1-based indexing
    clause_pointers.clear();
    clause_pointers.push_back(nullptr); // dummy for 1-based indexing
    for (auto& clause_vec : clause_storage) {
        clause_pointers.push_back(clause_vec.data());
    }
    
    probSAT_format_ready = true;
    
    std::cerr << "[CNFProbSAT] Generated ProbSAT format: " << clause_pointers.size()-1 
              << " clause pointers, " << total_clauses << " total clauses" << std::endl;
}

int** CNFProbSAT::get_clause_pointers() const {
    if (!probSAT_format_ready) {
        std::cerr << "[CNFProbSAT] ERROR: ProbSAT format not ready! Call generate_probSAT_format() first." << std::endl;
        return nullptr;
    }
    return const_cast<int**>(clause_pointers.data());
}

int CNFProbSAT::get_num_clauses() const {
    if (!probSAT_format_ready) {
        return 0;
    }
    return clause_pointers.size() - 1; // Subtract 1 for the dummy nullptr
}

int CNFProbSAT::get_num_variables() const {
    return count_variables();
}

void CNFProbSAT::clear_probSAT_format() {
    clause_pointers.clear();
    clause_storage.clear();
    probSAT_format_ready = false;
}

void CNFProbSAT::clear() {
    // Call base class clear first
    CNF::clear();
    // Then clear ProbSAT format
    clear_probSAT_format();
} 