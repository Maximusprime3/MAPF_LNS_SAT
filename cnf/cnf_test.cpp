#include <iostream>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <memory>
#include "CNFConstructor.h"
#include "../mdd/MDD.h"
#include "../mdd/MDDNode.h"
#include "MDDConstructor.h"
#include <set> // Added for set operations

// Helper to print the map
void print_map(const std::vector<std::string>& map) {
    for (const auto& row : map) {
        std::cout << row << std::endl;
    }
}

int main() {
    std::cout << "=== CNF Constructor Test (Char Map) ===" << std::endl;
    
    // 3x3 grid map
    std::vector<std::string> map = {
        "...",
        "...",
        "..."
    };
    std::cout << "Map:" << std::endl;
    print_map(map);
    
    // Agent 0: (0,0) -> (2,2) (diagonal from upper left to lower right)
    // Agent 1: (2,2) -> (0,0) (diagonal from lower right to upper left)
    // Agent 2: (1,0) -> (1,2) (vertical path in the middle)
    MDDNode::Position start0 = {0,0}, goal0 = {2,2};
    MDDNode::Position start1 = {2,2}, goal1 = {0,0};
    MDDNode::Position start2 = {1,0}, goal2 = {1,2};
    
    // Convert map to vector<vector<char>>
    std::vector<std::vector<char>> grid;
    for (const auto& row : map) grid.emplace_back(row.begin(), row.end());
    // Compute horizon (shortest path length for each agent)
    int horizon0 = abs(goal0.first - start0.first) + abs(goal0.second - start0.second);
    int horizon1 = abs(goal1.first - start1.first) + abs(goal1.second - start1.second);
    int horizon2 = abs(goal2.first - start2.first) + abs(goal2.second - start2.second);
    // Build MDDs using MDDConstructor
    MDDConstructor mdd_builder0(grid, start0, goal0, horizon0);
    auto mdd0 = mdd_builder0.construct_mdd();
    MDDConstructor mdd_builder1(grid, start1, goal1, horizon1);
    auto mdd1 = mdd_builder1.construct_mdd();
    MDDConstructor mdd_builder2(grid, start2, goal2, horizon2);
    auto mdd2 = mdd_builder2.construct_mdd();
    
    std::unordered_map<int, std::shared_ptr<MDD>> mdds;
    mdds[0] = mdd0;
    mdds[1] = mdd1;
    mdds[2] = mdd2;
    
    std::cout << "\nAgent 0: (" << start0.first << "," << start0.second << ") -> (" 
              << goal0.first << "," << goal0.second << ")" << std::endl;
    std::cout << "Agent 1: (" << start1.first << "," << start1.second << ") -> (" 
              << goal1.first << "," << goal1.second << ")" << std::endl;
    
    // Compare lazy vs eager encoding
    std::cout << "\n=== Lazy vs Eager Encoding Comparison ===" << std::endl;
    
    CNFConstructor lazy_constructor(mdds, true);  // true = lazy encoding
    CNF lazy_cnf = lazy_constructor.construct_cnf();
    std::cout << "Lazy encoding: " << lazy_cnf.count_clauses() << " clauses, " 
              << lazy_cnf.count_variables() << " variables" << std::endl;
    
    CNFConstructor eager_constructor(mdds, false); // false = eager encoding
    CNF eager_cnf = eager_constructor.construct_cnf();
    std::cout << "Eager encoding: " << eager_cnf.count_clauses() << " clauses, " 
              << eager_cnf.count_variables() << " variables" << std::endl;
    
    // Show additional clauses in eager encoding
    std::cout << "\n=== Additional Clauses in Eager Encoding ===" << std::endl;
    const auto& lazy_clauses = lazy_cnf.get_clauses();
    const auto& eager_clauses = eager_cnf.get_clauses();
    
    std::set<std::vector<int>> lazy_clause_set(lazy_clauses.begin(), lazy_clauses.end());
    std::set<std::vector<int>> eager_clause_set(eager_clauses.begin(), eager_clauses.end());
    
    int additional_count = 0;
    for (const auto& clause : eager_clauses) {
        if (lazy_clause_set.find(clause) == lazy_clause_set.end()) {
            std::cout << "Additional clause " << (++additional_count) << ": [";
            for (size_t i = 0; i < clause.size(); ++i) {
                std::cout << clause[i];
                if (i + 1 < clause.size()) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
    }
    
    if (additional_count == 0) {
        std::cout << "No additional clauses in eager encoding." << std::endl;
    } else {
        std::cout << "Total additional clauses: " << additional_count << std::endl;
    }
    
    // Test adding collision clauses to the same lazy CNF
    std::cout << "\n=== Testing Collision Clauses on Lazy CNF ===" << std::endl;
    
    // Add a valid collision clause (agents at same position and time)
    try {
        lazy_constructor.add_single_collision_clause(0, 1, MDDNode::Position(1,1), 1, true);
        std::cout << "✓ Added valid collision clause (agents 0,1 at (1,1), t=1)" << std::endl;
        std::cout << "  Lazy CNF now has: " << lazy_constructor.get_cnf().count_clauses() << " clauses, " 
                  << lazy_constructor.get_cnf().count_variables() << " variables" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "✗ Failed to add collision clause: " << e.what() << std::endl;
        std::cout << "  Lazy CNF still has: " << lazy_constructor.get_cnf().count_clauses() << " clauses, " 
                  << lazy_constructor.get_cnf().count_variables() << " variables" << std::endl;
    }
    
    // Try to add an invalid collision clause (position not in MDD)
    try {
        lazy_constructor.add_single_collision_clause(0, 1, MDDNode::Position(1,2), 0, true);
        std::cout << "✓ Added collision clause (agents 0,1 at (1,2), t=0)" << std::endl;
        std::cout << "  Lazy CNF now has: " << lazy_constructor.get_cnf().count_clauses() << " clauses, " 
                  << lazy_constructor.get_cnf().count_variables() << " variables" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "✗ Failed to add invalid collision clause: " << e.what() << std::endl;
        std::cout << "  Lazy CNF still has: " << lazy_constructor.get_cnf().count_clauses() << " clauses, " 
                  << lazy_constructor.get_cnf().count_variables() << " variables" << std::endl;
    }
    
    // Test adding edge collision clauses to the same lazy CNF
    std::cout << "\n=== Testing Edge Collision Clauses on Lazy CNF ===" << std::endl;
    
    // Add a valid edge collision clause (agents swap positions on valid edge)
    try {
        lazy_constructor.add_single_edge_collision_clause(0, 1, MDDNode::Position(1,1), MDDNode::Position(1,2), 1, true);
        std::cout << "✓ Added valid edge collision clause (agents 0,1 swap (1,1)<->(1,2), t=1)" << std::endl;
        std::cout << "  Lazy CNF now has: " << lazy_constructor.get_cnf().count_clauses() << " clauses, " 
                  << lazy_constructor.get_cnf().count_variables() << " variables" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "✗ Failed to add edge collision clause: " << e.what() << std::endl;
        std::cout << "  Lazy CNF still has: " << lazy_constructor.get_cnf().count_clauses() << " clauses, " 
                  << lazy_constructor.get_cnf().count_variables() << " variables" << std::endl;
    }
    
    // Try to add an invalid edge collision clause (edge not in MDD)
    try {
        lazy_constructor.add_single_edge_collision_clause(0, 1, MDDNode::Position(0,0), MDDNode::Position(2,2), 0, true);
        std::cout << "✓ Added edge collision clause (agents 0,1 swap (0,0)<->(2,2), t=0)" << std::endl;
        std::cout << "  Lazy CNF now has: " << lazy_constructor.get_cnf().count_clauses() << " clauses, " 
                  << lazy_constructor.get_cnf().count_variables() << " variables" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "✗ Failed to add invalid edge collision clause: " << e.what() << std::endl;
        std::cout << "  Lazy CNF still has: " << lazy_constructor.get_cnf().count_clauses() << " clauses, " 
                  << lazy_constructor.get_cnf().count_variables() << " variables" << std::endl;
    }
    
    // Final summary
    std::cout << "\n=== Final Summary ===" << std::endl;
    std::cout << "Final lazy CNF has: " << lazy_constructor.get_cnf().count_clauses() << " clauses, " 
              << lazy_constructor.get_cnf().count_variables() << " variables" << std::endl;
    
    return 0;
} 