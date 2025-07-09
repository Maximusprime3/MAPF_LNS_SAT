#ifndef CNFPROBSATCONSTRUCTOR_H
#define CNFPROBSATCONSTRUCTOR_H

#include "CNFConstructor.h"
#include <vector>

// CNFProbSATConstructor extends CNFConstructor to generate CNF directly in ProbSAT format
// It stores clauses in the format expected by ProbSAT's in-memory API
class CNFProbSATConstructor : public CNFConstructor {
private:
    // ProbSAT-specific storage for clauses
    std::vector<std::vector<int>> probsat_clause_storage;
    std::vector<int*> probsat_clause_pointers;

public:
    // Constructor: initializes with MDDs and optional parameters
    CNFProbSATConstructor(const std::unordered_map<int, std::shared_ptr<MDD>>& mdds, 
                          bool lazy_encoding = false,
                          const std::unordered_map<std::tuple<int, MDDNode::Position, int>, int>& existing_variable_map = {},
                          int start_variable_id = 1);

    // Override add_clause to store in ProbSAT format (null-terminated)
    void add_clause(const std::vector<int>& clause);

    // Main method to construct the complete CNF from MDDs in ProbSAT format
    void construct_probsat_cnf(const std::vector<std::tuple<int, int, MDDNode::Position, int>>& collisions = {});

    // ProbSAT accessors for the in-memory API
    int** get_probsat_clause_pointers();
    int get_probsat_num_clauses() const;
    int get_probsat_num_variables() const;

private:
    // Build the clause pointers array for ProbSAT
    void build_clause_pointers();
};

#endif // CNFPROBSATCONSTRUCTOR_H 