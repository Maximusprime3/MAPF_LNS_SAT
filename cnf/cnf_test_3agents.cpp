#include <iostream>
#include <vector>
#include <tuple>
#include <unordered_map>
#include <memory>
#include <random>
#include "CNFConstructor.h"
#include "../mdd/MDD.h"
#include "../mdd/MDDNode.h"
#include "MDDConstructor.h"

// Helper to print the map
void print_map(const std::vector<std::string>& map) {
    for (const auto& row : map) {
        std::cout << row << std::endl;
    }
}

int main() {
    std::cout << "=== CNF Constructor Test with 3 Agents ===" << std::endl;
    
    // 4x4 grid map with some obstacles
    std::vector<std::string> map = {
        "....",
        ".@..",
        "..@.",
        "...."
    };
    std::cout << "Map ('.' = free, '@' = obstacle):" << std::endl;
    print_map(map);
    
    // Define start and goal positions for 3 agents
    MDDNode::Position start0 = {0, 0}, goal0 = {3, 3};  // Agent 0: top-left to bottom-right
    MDDNode::Position start1 = {3, 0}, goal1 = {0, 3};  // Agent 1: bottom-left to top-right  
    MDDNode::Position start2 = {1, 0}, goal2 = {2, 3};  // Agent 2: middle area
    
    std::cout << "\nAgent positions:" << std::endl;
    std::cout << "Agent 0: (" << start0.first << "," << start0.second << ") -> (" 
              << goal0.first << "," << goal0.second << ")" << std::endl;
    std::cout << "Agent 1: (" << start1.first << "," << start1.second << ") -> (" 
              << goal1.first << "," << goal1.second << ")" << std::endl;
    std::cout << "Agent 2: (" << start2.first << "," << start2.second << ") -> (" 
              << goal2.first << "," << goal2.second << ")" << std::endl;
    
    // Convert map to vector<vector<char>>
    std::vector<std::vector<char>> grid;
    for (const auto& row : map) grid.emplace_back(row.begin(), row.end());
    
    // Compute horizons (shortest path length for each agent)
    // Use a more generous horizon to account for obstacles
    int horizon0 = abs(goal0.first - start0.first) + abs(goal0.second - start0.second) + 2;
    int horizon1 = abs(goal1.first - start1.first) + abs(goal1.second - start1.second) + 2;
    int horizon2 = abs(goal2.first - start2.first) + abs(goal2.second - start2.second) + 4; // More generous for Agent 2
    
    std::cout << "\nHorizons: Agent 0=" << horizon0 << ", Agent 1=" << horizon1 << ", Agent 2=" << horizon2 << std::endl;
    
    // Build MDDs using MDDConstructor
    std::cout << "\nBuilding MDDs..." << std::endl;
    
    MDDConstructor mdd_builder0(grid, start0, goal0, horizon0);
    auto mdd0 = mdd_builder0.construct_mdd();
    
    MDDConstructor mdd_builder1(grid, start1, goal1, horizon1);
    auto mdd1 = mdd_builder1.construct_mdd();
    
    MDDConstructor mdd_builder2(grid, start2, goal2, horizon2);
    auto mdd2 = mdd_builder2.construct_mdd();
    
    // Create MDD map
    std::unordered_map<int, std::shared_ptr<MDD>> mdds;
    mdds[0] = mdd0;
    mdds[1] = mdd1;
    mdds[2] = mdd2;
    
    // Print MDD info
    std::cout << "\nMDD Information:" << std::endl;
    for (const auto& agent_mdd_pair : mdds) {
        int agent_id = agent_mdd_pair.first;
        const auto& mdd = agent_mdd_pair.second;
        std::cout << "Agent " << agent_id << ": " << mdd->levels.size() << " timesteps" << std::endl;
        
        // Count total nodes
        int total_nodes = 0;
        for (const auto& level_pair : mdd->levels) {
            total_nodes += level_pair.second.size();
        }
        std::cout << "  Total nodes: " << total_nodes << std::endl;
    }
    
    // Create CNF from MDDs
    std::cout << "\n=== Creating CNF from MDDs ===" << std::endl;
    
    // Try both lazy and eager encoding
    CNFConstructor lazy_constructor(mdds, true);  // true = lazy encoding
    CNF lazy_cnf = lazy_constructor.construct_cnf();
    
    CNFConstructor eager_constructor(mdds, false); // false = eager encoding
    CNF eager_cnf = eager_constructor.construct_cnf();
    
    std::cout << "Lazy encoding: " << lazy_cnf.count_clauses() << " clauses, " 
              << lazy_cnf.count_variables() << " variables" << std::endl;
    std::cout << "Eager encoding: " << eager_cnf.count_clauses() << " clauses, " 
              << eager_cnf.count_variables() << " variables" << std::endl;
    
    // Show some example clauses
    std::cout << "\n=== Example Clauses (Lazy Encoding) ===" << std::endl;
    const auto& clauses = lazy_cnf.get_clauses();
    int shown = 0;
    for (const auto& clause : clauses) {
        if (shown < 5) {  // Show first 5 clauses
            std::cout << "Clause " << (shown + 1) << ": [";
            for (size_t i = 0; i < clause.size(); ++i) {
                std::cout << clause[i];
                if (i + 1 < clause.size()) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            shown++;
        } else {
            break;
        }
    }
    if (clauses.size() > 5) {
        std::cout << "... and " << (clauses.size() - 5) << " more clauses" << std::endl;
    }
    
    // Test path validation
    std::cout << "\n=== Testing Path Validation ===" << std::endl;
    
    // Generate sample paths from MDDs using built-in function
    std::random_device rd;
    std::mt19937 rng(rd());
    
    std::vector<MDDNode::Position> sample_path0 = mdd0->sample_random_path(rng);
    std::vector<MDDNode::Position> sample_path1 = mdd1->sample_random_path(rng);
    std::vector<MDDNode::Position> sample_path2 = mdd2->sample_random_path(rng);
    
    bool valid0 = lazy_constructor.validate_path(0, sample_path0);
    bool valid1 = lazy_constructor.validate_path(1, sample_path1);
    bool valid2 = lazy_constructor.validate_path(2, sample_path2);
    
    std::cout << "Sample path for Agent 0: ";
    for (const auto& pos : sample_path0) {
        std::cout << "(" << pos.first << "," << pos.second << ") ";
    }
    std::cout << "-> " << (valid0 ? "VALID" : "INVALID") << std::endl;
    
    std::cout << "Sample path for Agent 1: ";
    for (const auto& pos : sample_path1) {
        std::cout << "(" << pos.first << "," << pos.second << ") ";
    }
    std::cout << "-> " << (valid1 ? "VALID" : "INVALID") << std::endl;
    
    std::cout << "Sample path for Agent 2: ";
    for (const auto& pos : sample_path2) {
        std::cout << "(" << pos.first << "," << pos.second << ") ";
    }
    std::cout << "-> " << (valid2 ? "VALID" : "INVALID") << std::endl;
    
    // Test CNF assignment conversion
    std::cout << "\n=== Testing CNF Assignment Conversion ===" << std::endl;
    
    std::vector<int> assignment = lazy_constructor.path_to_cnf_assignment(0, sample_path0);
    std::cout << "CNF assignment for Agent 0 path: [";
    for (size_t i = 0; i < assignment.size(); ++i) {
        std::cout << assignment[i];
        if (i + 1 < assignment.size()) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    // Convert back to paths
    auto agent_paths = lazy_constructor.cnf_assignment_to_paths(assignment);
    std::cout << "Converted back to paths:" << std::endl;
    for (const auto& agent_path_pair : agent_paths) {
        int agent_id = agent_path_pair.first;
        const auto& path = agent_path_pair.second;
        std::cout << "Agent " << agent_id << ": ";
        for (const auto& pos : path) {
            std::cout << "(" << pos.first << "," << pos.second << ") ";
        }
        std::cout << std::endl;
    }
    
    std::cout << "\n=== Test Complete ===" << std::endl;
    
    return 0;
} 