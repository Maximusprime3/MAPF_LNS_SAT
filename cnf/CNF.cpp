#include "CNF.h"
#include <sstream>
#include <algorithm>

CNF::CNF() {
    // Constructor initializes empty CNF
}

void CNF::add_clause(const std::vector<int>& clause) {
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