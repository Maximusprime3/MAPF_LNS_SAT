#ifndef CNFPROBSAT_H
#define CNFPROBSAT_H

#include "CNF.h"
#include <vector>

// CNFProbSAT extends CNF with ProbSAT-specific functionality
// It can generate CNF directly in ProbSAT's expected format
class CNFProbSAT : public CNF {
private:
    std::vector<int*> clause_pointers;
    std::vector<std::vector<int>> clause_storage;
    bool probSAT_format_ready;

public:
    // Constructor
    CNFProbSAT();
    
    // Destructor to clean up allocated memory
    ~CNFProbSAT();
    
    // Generate ProbSAT format directly (no conversion needed)
    void generate_probSAT_format();
    
    // Get ProbSAT-compatible data structures
    int** get_clause_pointers() const;
    int get_num_clauses() const;
    int get_num_variables() const;
    
    // Check if ProbSAT format is ready
    bool is_probSAT_ready() const { return probSAT_format_ready; }
    
    // Clear ProbSAT format data
    void clear_probSAT_format();
    
    // Clear both CNF and ProbSAT format data
    void clear();
};

#endif // CNFPROBSAT_H 