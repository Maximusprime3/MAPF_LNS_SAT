#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <deque>
#include <algorithm>

// Include our MAPF components
#include "mdd/MDDConstructor.h"
#include "mdd/MDD.h"
#include "mdd/MDDNode.h"
#include "cnf/CNFConstructor.h"
#include "cnf/CNF.h"

// Helper function to load map from file
std::vector<std::vector<char>> load_map(const std::string& map_path) {
    std::ifstream file(map_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open map file: " + map_path);
    }
    
    std::string line;
    std::vector<std::vector<char>> grid;
    
    // Skip header lines
    while (std::getline(file, line)) {
        if (line == "map") {
            break;
        }
    }
    
    // Read the actual map
    while (std::getline(file, line)) {
        if (line.empty()) break;
        std::vector<char> row;
        for (char c : line) {
            if (c == '.' || c == '@' || c == 'T' || c == 'S' || c == 'W' || c == 'G') {
                row.push_back(c);
            }
        }
        if (!row.empty()) {
            grid.push_back(row);
        }
    }
    
    return grid;
}

// Helper function to load scenario and extract agent starts/goals
std::vector<std::pair<MDDNode::Position, MDDNode::Position>> 
load_scenario_agents(const std::string& scenario_path, int num_agents) {
    std::ifstream file(scenario_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open scenario file: " + scenario_path);
    }
    
    std::string line;
    std::vector<std::pair<MDDNode::Position, MDDNode::Position>> agents;
    
    // Skip version line
    std::getline(file, line);
    
    // Read agent data
    int agent_count = 0;
    while (std::getline(file, line) && agent_count < num_agents) {
        std::istringstream iss(line);
        int agent_id, map_w, map_h, start_x, start_y, goal_x, goal_y;
        std::string map_name;
        double distance;
        
        if (iss >> agent_id >> map_name >> map_w >> map_h >> start_x >> start_y >> goal_x >> goal_y >> distance) {
            agents.push_back({{start_x, start_y}, {goal_x, goal_y}});
            agent_count++;
        }
    }
    
    return agents;
}

int main() {
    try {
        std::cout << "=== CNF Debug Analysis ===" << std::endl;
        
        // Configuration
        const std::string map_path = "mapf-map/empty-16-16.map";
        const std::string scenario_path = "mapf-scen-even/scen-even/empty-16-16-even-1.scen";
        const int num_agents = 1;
        const int max_timesteps = 3;
        
        std::cout << "Loading map from: " << map_path << std::endl;
        auto grid = load_map(map_path);
        std::cout << "Map loaded: " << grid.size() << "x" << grid[0].size() << std::endl;
        
        std::cout << "Loading scenario from: " << scenario_path << std::endl;
        auto agents = load_scenario_agents(scenario_path, num_agents);
        // Manually override the goal for agent 0 to (10,5) for testing
        if (!agents.empty()) {
            agents[0].second = {10, 5};
        }
        std::cout << "Loaded " << agents.size() << " agents" << std::endl;
        
        // Print agent information
        for (size_t i = 0; i < agents.size(); ++i) {
            std::cout << "Agent " << i << ": Start(" << agents[i].first.first << "," << agents[i].first.second 
                      << ") -> Goal(" << agents[i].second.first << "," << agents[i].second.second << ")" << std::endl;
        }
        
        // Step 1: Create MDD for the agent
        std::cout << "\n=== Creating MDD ===" << std::endl;
        MDDConstructor constructor(grid, agents[0].first, agents[0].second, max_timesteps);
        auto mdd = constructor.construct_mdd();
        
        // Check if MDD is valid
        if (mdd->is_broken(max_timesteps)) {
            std::cerr << "ERROR: MDD is broken!" << std::endl;
            return 1;
        }
        
        std::cout << "MDD has " << mdd->levels.size() << " timesteps" << std::endl;
        
        // Print MDD structure in detail
        std::cout << "\n=== MDD Structure ===" << std::endl;
        for (const auto& [level, nodes] : mdd->levels) {
            std::cout << "Level " << level << ": ";
            for (const auto& node : nodes) {
                std::cout << "(" << node->position.first << "," << node->position.second << ") ";
            }
            std::cout << std::endl;
        }
        
        // Print parent-child relationships
        mdd->print_node_relationships();
        
        // Step 2: Create CNF from MDD
        std::cout << "\n=== Creating CNF ===" << std::endl;
        std::unordered_map<int, std::shared_ptr<MDD>> mdd_map;
        mdd_map[0] = mdd;
        
        CNFConstructor cnf_constructor(mdd_map, false); // false = eager encoding
        CNF cnf = cnf_constructor.construct_cnf();

        // Print the variable map for debugging
        const auto& var_map = cnf_constructor.get_variable_map();
        std::cout << "\n========== CNF VARIABLE MAP ==========" << std::endl;
        for (const auto& [key, var_id] : var_map) {
            int agent_id = std::get<0>(key);
            auto pos = std::get<1>(key);
            int timestep = std::get<2>(key);
            std::cout << "  " << var_id << ": agent " << agent_id << ", pos (" << pos.first << "," << pos.second << "), t=" << timestep << std::endl;
        }
        std::cout << "========== END CNF VARIABLE MAP ==========" << std::endl;
        
        std::cout << "CNF created: " << cnf.count_variables() << " variables, " 
                  << cnf.count_clauses() << " clauses" << std::endl;
        
        // Print all clauses
        const auto& clauses = cnf.get_clauses();
        std::cout << "\n========== ALL CNF CLAUSES ==========" << std::endl;
        for (size_t i = 0; i < clauses.size(); ++i) {
            std::cout << "Clause " << (i+1) << ": ";
            for (int lit : clauses[i]) {
                std::cout << lit << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "========== END ALL CNF CLAUSES ==========" << std::endl;
        
        // Test path extraction
        std::cout << "\n=== Testing Path Extraction ===" << std::endl;
        
        // Create a test assignment where all variables are true
        std::vector<int> test_assignment;
        for (int i = 1; i <= cnf.count_variables(); ++i) {
            test_assignment.push_back(i);
        }
        
        std::cout << "Test assignment (all variables true): ";
        for (int v : test_assignment) std::cout << v << " ";
        std::cout << std::endl;
        
        auto agent_paths = cnf_constructor.cnf_assignment_to_paths(test_assignment);
        std::cout << "Extracted paths:" << std::endl;
        for (const auto& [agent_id, path] : agent_paths) {
            std::cout << "  Agent " << agent_id << ": ";
            for (const auto& pos : path) {
                std::cout << "(" << pos.first << "," << pos.second << ") ";
            }
            std::cout << std::endl;
        }
        
        // Test path validation
        std::cout << "\n=== Testing Path Validation ===" << std::endl;
        for (const auto& [agent_id, path] : agent_paths) {
            bool valid = cnf_constructor.validate_path(agent_id, path);
            std::cout << "Agent " << agent_id << " path is " 
                      << (valid ? "VALID" : "INVALID") << std::endl;
        }
        
        std::cout << "\n=== Debug completed ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 