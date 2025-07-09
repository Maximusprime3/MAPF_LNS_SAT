#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <tuple>

// Mock the types we need
using Position = std::pair<int, int>;
using VariableKey = std::tuple<int, Position, int>;

int main() {
    // Simulate the variable map from our test
    std::unordered_map<VariableKey, int> variable_map;
    variable_map[{0, {10, 5}, 3}] = 1;  // Variable 1: agent 0, pos (10,5), t=3
    variable_map[{0, {10, 6}, 2}] = 2;  // Variable 2: agent 0, pos (10,6), t=2
    variable_map[{0, {10, 7}, 1}] = 3;  // Variable 3: agent 0, pos (10,7), t=1
    variable_map[{0, {10, 8}, 0}] = 4;  // Variable 4: agent 0, pos (10,8), t=0
    
    // Simulate the assignment [1, 1, 1, 1] (all variables true)
    std::vector<int> assignment = {1, 1, 1, 1};
    
    // Group positive assignments by agent
    std::unordered_map<int, std::vector<std::pair<int, Position>>> agent_positions;
    
    std::cout << "Processing assignment: ";
    for (int v : assignment) std::cout << v << " ";
    std::cout << std::endl;
    
    for (int var_id : assignment) {
        if (var_id > 0) { // Only positive assignments
            std::cout << "Processing variable " << var_id << std::endl;
            
            // Find the (agent, position, timestep) for this variable
            for (const auto& var_pair : variable_map) {
                if (var_pair.second == var_id) {
                    int agent_id = std::get<0>(var_pair.first);
                    Position position = std::get<1>(var_pair.first);
                    int timestep = std::get<2>(var_pair.first);
                    
                    std::cout << "  Found: agent " << agent_id 
                              << ", pos (" << position.first << "," << position.second 
                              << "), t=" << timestep << std::endl;
                    
                    agent_positions[agent_id].push_back({timestep, position});
                    break;
                }
            }
        }
    }
    
    // Convert to paths for each agent
    for (const auto& agent_pos_pair : agent_positions) {
        int agent_id = agent_pos_pair.first;
        const auto& positions = agent_pos_pair.second;
        
        std::cout << "\nAgent " << agent_id << " positions before sorting:" << std::endl;
        for (const auto& pos_pair : positions) {
            std::cout << "  t=" << pos_pair.first << ": (" << pos_pair.second.first 
                      << "," << pos_pair.second.second << ")" << std::endl;
        }
        
        // Sort by timestep
        std::vector<std::pair<int, Position>> sorted_positions = positions;
        std::sort(sorted_positions.begin(), sorted_positions.end());
        
        std::cout << "Agent " << agent_id << " positions after sorting:" << std::endl;
        for (const auto& pos_pair : sorted_positions) {
            std::cout << "  t=" << pos_pair.first << ": (" << pos_pair.second.first 
                      << "," << pos_pair.second.second << ")" << std::endl;
        }
        
        // Extract path
        std::vector<Position> path;
        for (const auto& pos_pair : sorted_positions) {
            path.push_back(pos_pair.second);
        }
        
        std::cout << "Final path: ";
        for (const auto& pos : path) {
            std::cout << "(" << pos.first << "," << pos.second << ") ";
        }
        std::cout << std::endl;
    }
    
    return 0;
} 