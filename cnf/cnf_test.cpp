#include "CNFConstructor.h"
#include "../mdd/MDDConstructor.h"
#include <iostream>
#include <memory>
#include <fstream>

int main() {
    std::cout << "CNF Test Program" << std::endl;
    std::cout << "=================" << std::endl;
    
    // Create a grid similar to main.cpp
    // '.' = open cell, 'G' = goal, '#' = obstacle
    std::vector<std::vector<char>> grid = {
        {'.', '.', '.'},
        {'.', '#', '.'},  // Center obstacle
        {'.', '.', 'G'}
    };
    
    std::cout << "Grid layout:" << std::endl;
    for (const auto& row : grid) {
        for (char cell : row) {
            std::cout << cell << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // Create MDDs for two agents
    std::unordered_map<int, std::shared_ptr<MDD>> mdds;
    
    // Agent 0: from (0,0) to (2,2)
    auto start0 = std::make_pair(0, 0);
    auto goal0 = std::make_pair(2, 2);
    std::cout << "Creating MDD for Agent 0: (" << start0.first << "," << start0.second 
              << ") -> (" << goal0.first << "," << goal0.second << ")" << std::endl;
    MDDConstructor mdd_constructor0(grid, start0, goal0, 6);
    mdds[0] = mdd_constructor0.construct_mdd();
    
    // Agent 1: from (2,0) to (0,2)
    auto start1 = std::make_pair(2, 0);
    auto goal1 = std::make_pair(0, 2);
    std::cout << "Creating MDD for Agent 1: (" << start1.first << "," << start1.second 
              << ") -> (" << goal1.first << "," << goal1.second << ")" << std::endl;
    MDDConstructor mdd_constructor1(grid, start1, goal1, 6);
    mdds[1] = mdd_constructor1.construct_mdd();
    
    // Print MDD information
    std::cout << "\nMDD Information:" << std::endl;
    for (const auto& agent_mdd_pair : mdds) {
        int agent_id = agent_mdd_pair.first;
        const auto& mdd = agent_mdd_pair.second;
        std::cout << "Agent " << agent_id << " MDD:" << std::endl;
        for (const auto& level_pair : mdd->levels) {
            int timestep = level_pair.first;
            const auto& nodes = level_pair.second;
            std::cout << "  Level " << timestep << " (" << nodes.size() << " nodes): ";
            for (const auto& node : nodes) {
                std::cout << "(" << node->position.first << "," << node->position.second << ") ";
            }
            std::cout << std::endl;
        }
    }
    
    // Create CNF constructor
    std::cout << "\nCreating CNF constructor..." << std::endl;
    CNFConstructor cnf_constructor(mdds, false); // Non-lazy encoding
    
    // Construct CNF
    std::cout << "Constructing CNF..." << std::endl;
    CNF cnf = cnf_constructor.construct_cnf();
    
    std::cout << "\nCNF Statistics:" << std::endl;
    std::cout << "Variables: " << cnf.count_variables() << std::endl;
    std::cout << "Clauses: " << cnf.count_clauses() << std::endl;
    
    // Print first few clauses as example
    std::cout << "\nFirst 10 clauses (DIMACS format):" << std::endl;
    auto clauses = cnf.get_clauses();
    for (int i = 0; i < std::min(10, (int)clauses.size()); ++i) {
        std::cout << "Clause " << i + 1 << ": ";
        for (int literal : clauses[i]) {
            std::cout << literal << " ";
        }
        std::cout << "0" << std::endl;
    }
    
    // Test lazy encoding
    std::cout << "\nTesting lazy encoding..." << std::endl;
    CNFConstructor lazy_constructor(mdds, true); // Lazy encoding
    CNF lazy_cnf = lazy_constructor.construct_cnf();
    
    std::cout << "Lazy CNF Statistics:" << std::endl;
    std::cout << "Variables: " << lazy_cnf.count_variables() << std::endl;
    std::cout << "Clauses: " << lazy_cnf.count_clauses() << std::endl;
    std::cout << "Clause reduction: " << (cnf.count_clauses() - lazy_cnf.count_clauses()) 
              << " clauses saved with lazy encoding" << std::endl;
    
    // Test adding specific collision clauses
    std::cout << "\nTesting collision clause addition..." << std::endl;
    std::vector<std::tuple<int, int, MDDNode::Position, int>> collisions = {
        {0, 1, {1, 0}, 2},  // Agents 0 and 1 cannot both be at (1,0) at timestep 2
        {0, 1, {0, 1}, 3}   // Agents 0 and 1 cannot both be at (0,1) at timestep 3
    };
    
    // Create a copy of the original CNF and add collision clauses to it
    CNF collision_cnf = cnf;
    cnf_constructor.add_collision_clauses_to_cnf(collision_cnf, collisions);
    
    std::cout << "CNF with collision clauses:" << std::endl;
    std::cout << "Variables: " << collision_cnf.count_variables() << std::endl;
    std::cout << "Clauses: " << collision_cnf.count_clauses() << std::endl;
    std::cout << "Additional clauses: " << (collision_cnf.count_clauses() - cnf.count_clauses()) << std::endl;
    
    // Save CNF to file in DIMACS format
    std::cout << "\nSaving CNF to file 'test.cnf'..." << std::endl;
    std::ofstream outfile("test.cnf");
    if (outfile.is_open()) {
        outfile << cnf.to_dimacs();
        outfile.close();
        std::cout << "CNF saved successfully!" << std::endl;
    } else {
        std::cout << "Failed to save CNF file!" << std::endl;
    }
    
    // Save lazy CNF to file
    std::cout << "Saving lazy CNF to file 'test_lazy.cnf'..." << std::endl;
    std::ofstream lazy_outfile("test_lazy.cnf");
    if (lazy_outfile.is_open()) {
        lazy_outfile << lazy_cnf.to_dimacs();
        lazy_outfile.close();
        std::cout << "Lazy CNF saved successfully!" << std::endl;
    } else {
        std::cout << "Failed to save lazy CNF file!" << std::endl;
    }
    
    // Test path-to-CNF and CNF-to-path translation
    std::cout << "\n=== Testing Path-CNF Translation ===" << std::endl;
    
    // Create sample paths for testing
    std::vector<MDDNode::Position> path0 = {
        {0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2}  // Agent 0 path
    };
    std::vector<MDDNode::Position> path1 = {
        {2, 0}, {1, 0}, {0, 0}, {0, 1}, {0, 2}  // Agent 1 path
    };
    
    std::cout << "Sample paths:" << std::endl;
    std::cout << "Agent 0: ";
    for (const auto& pos : path0) {
        std::cout << "(" << pos.first << "," << pos.second << ") ";
    }
    std::cout << std::endl;
    
    std::cout << "Agent 1: ";
    for (const auto& pos : path1) {
        std::cout << "(" << pos.first << "," << pos.second << ") ";
    }
    std::cout << std::endl;
    
    // Test path validation
    std::cout << "\nPath validation:" << std::endl;
    bool valid0 = cnf_constructor.validate_path(0, path0);
    bool valid1 = cnf_constructor.validate_path(1, path1);
    std::cout << "Agent 0 path valid: " << (valid0 ? "YES" : "NO") << std::endl;
    std::cout << "Agent 1 path valid: " << (valid1 ? "YES" : "NO") << std::endl;
    
    // Test path-to-CNF translation
    std::cout << "\nPath-to-CNF translation:" << std::endl;
    std::vector<int> assignment0 = cnf_constructor.path_to_cnf_assignment(0, path0);
    std::vector<int> assignment1 = cnf_constructor.path_to_cnf_assignment(1, path1);
    
    std::cout << "Agent 0 CNF variables: ";
    for (int var : assignment0) {
        std::cout << var << " ";
    }
    std::cout << std::endl;
    
    std::cout << "Agent 1 CNF variables: ";
    for (int var : assignment1) {
        std::cout << var << " ";
    }
    std::cout << std::endl;
    
    // Combine assignments for CNF-to-path translation
    std::vector<int> combined_assignment = assignment0;
    combined_assignment.insert(combined_assignment.end(), assignment1.begin(), assignment1.end());
    
    // Test CNF-to-path translation
    std::cout << "\nCNF-to-path translation:" << std::endl;
    auto extracted_paths = cnf_constructor.cnf_assignment_to_paths(combined_assignment);
    
    for (const auto& agent_path_pair : extracted_paths) {
        int agent_id = agent_path_pair.first;
        const auto& extracted_path = agent_path_pair.second;
        
        std::cout << "Agent " << agent_id << " extracted path: ";
        for (const auto& pos : extracted_path) {
            std::cout << "(" << pos.first << "," << pos.second << ") ";
        }
        std::cout << std::endl;
    }
    
    // Test round-trip consistency
    std::cout << "\nRound-trip consistency test:" << std::endl;
    bool round_trip_consistent = true;
    
    for (const auto& agent_path_pair : extracted_paths) {
        int agent_id = agent_path_pair.first;
        const auto& extracted_path = agent_path_pair.second;
        
        // Get original path
        const auto& original_path = (agent_id == 0) ? path0 : path1;
        
        if (extracted_path.size() != original_path.size()) {
            round_trip_consistent = false;
            std::cout << "Agent " << agent_id << ": Path length mismatch!" << std::endl;
            continue;
        }
        
        for (size_t i = 0; i < extracted_path.size(); ++i) {
            if (extracted_path[i] != original_path[i]) {
                round_trip_consistent = false;
                std::cout << "Agent " << agent_id << ": Position mismatch at step " << i 
                          << " - expected (" << original_path[i].first << "," << original_path[i].second 
                          << "), got (" << extracted_path[i].first << "," << extracted_path[i].second << ")" << std::endl;
            }
        }
    }
    
    std::cout << "Round-trip consistency: " << (round_trip_consistent ? "PASSED" : "FAILED") << std::endl;
    
    // Test invalid path
    std::cout << "\nTesting invalid path:" << std::endl;
    std::vector<MDDNode::Position> invalid_path = {
        {0, 0}, {1, 1}, {2, 2}  // Invalid: goes through obstacle at (1,1)
    };
    bool invalid_valid = cnf_constructor.validate_path(0, invalid_path);
    std::cout << "Invalid path validation: " << (invalid_valid ? "YES" : "NO") << " (should be NO)" << std::endl;
    
    return 0;
} 