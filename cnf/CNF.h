#ifndef CNF_H
#define CNF_H

#include <vector>
#include <set>
#include <string>
#include <unordered_map>

// CNF class represents a Conjunctive Normal Form formula
// It manages clauses and provides DIMACS format output
class CNF {
private:
    std::vector<std::vector<int>> clauses; // Each clause is a vector of literals
    std::set<int> variables; // Set of all variables used (for counting)

public:
    // Constructor: initializes an empty CNF
    CNF();
    
    // Adds a clause to the CNF formula
    void add_clause(const std::vector<int>& clause);
    
    // Returns the number of unique variables in the CNF
    int count_variables() const;
    
    // Returns the number of clauses
    int count_clauses() const;
    
    // Returns the CNF in DIMACS format as a string
    std::string to_dimacs() const;
    
    // Returns just the metadata line for DIMACS format
    std::string get_metadata() const;
    
    // Returns all clauses
    const std::vector<std::vector<int>>& get_clauses() const;
    
    // Clears all clauses and variables
    void clear();
};

#endif // CNF_H 