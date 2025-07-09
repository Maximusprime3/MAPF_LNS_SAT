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

// Include ProbSAT in-memory API
extern "C" {
    #include "probSAT-master/probSAT_inmem.h"
}

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

// Helper function to print agent paths
void print_agent_paths(const std::unordered_map<int, std::vector<MDDNode::Position>>& agent_paths) {
    std::cout << "\n=== Agent Paths ===" << std::endl;
    for (const auto& [agent_id, path] : agent_paths) {
        std::cout << "Agent " << agent_id << " path:" << std::endl;
        for (size_t t = 0; t < path.size(); ++t) {
            std::cout << "  Time " << t << ": (" << path[t].first << ", " << path[t].second << ")" << std::endl;
        }
        std::cout << std::endl;
    }
}

int main() {
    try {
        std::cout << "=== MAPF LNS SAT Orchestration Test ===" << std::endl;
        
        // Configuration
        const std::string map_path = "mapf-map/empty-16-16.map";
        const std::string scenario_path = "mapf-scen-even/scen-even/empty-16-16-even-1.scen";
        const int num_agents = 2;
        const int max_timesteps = 3; // Increased from 3 to 4 for more flexibility
        const long long seed = 42;
        const long long max_runs = 1;
        const long long max_flips = 10000;
        
        std::cout << "Loading map from: " << map_path << std::endl;
        auto grid = load_map(map_path);
        std::cout << "Map loaded: " << grid.size() << "x" << grid[0].size() << std::endl;
        
        std::cout << "Loading scenario from: " << scenario_path << std::endl;
        auto agents = load_scenario_agents(scenario_path, num_agents);
        // Manually override the goals for testing
        if (agents.size() >= 2) {
            // Agent 0: start and goal as before
            agents[0].first = {1, 1};
            agents[0].second = {1, 4};
            // Agent 1: start = agent 0's goal, goal = agent 0's start
            agents[1].first = agents[0].second;
            agents[1].second = agents[0].first;
        }
        std::cout << "Loaded " << agents.size() << " agents" << std::endl;
        
        // Print agent information
        for (size_t i = 0; i < agents.size(); ++i) {
            std::cout << "Agent " << i << ": Start(" << agents[i].first.first << "," << agents[i].first.second 
                      << ") -> Goal(" << agents[i].second.first << "," << agents[i].second.second << ")" << std::endl;
        }
        
        // Step 1: Create MDDs for each agent
        std::cout << "\n=== Creating MDDs ===" << std::endl;
        std::vector<std::shared_ptr<MDD>> mdds;
        std::unordered_map<int, std::shared_ptr<MDD>> mdd_map;
        
        for (int i = 0; i < num_agents; ++i) {
            std::cout << "Building MDD for Agent " << i << "..." << std::endl;
            
            MDDConstructor constructor(grid, agents[i].first, agents[i].second, max_timesteps);
            auto mdd = constructor.construct_mdd();
            
            // Check if MDD is valid
            if (mdd->is_broken(max_timesteps)) {
                std::cerr << "ERROR: MDD for Agent " << i << " is broken!" << std::endl;
                return 1;
            }
            
            mdds.push_back(mdd);
            mdd_map[i] = mdd;

            std::cout << "  MDD has " << mdd->levels.size() << " timesteps" << std::endl;
            // Print all nodes at each level for debugging
            for (const auto& [level, nodes] : mdd->levels) {
                std::cout << "    Level " << level << ": ";
                for (const auto& node : nodes) {
                    std::cout << "(" << node->position.first << "," << node->position.second << ") ";
                }
                std::cout << std::endl;
            }
            
            // Count total nodes
            int total_nodes = 0;
            for (const auto& [level, nodes] : mdd->levels) {
                total_nodes += nodes.size();
            }
            std::cout << "  Total nodes: " << total_nodes << std::endl;
        }
        
        // Step 2: Create CNF from MDDs
        std::cout << "\n=== Creating CNF ===" << std::endl;
        CNFConstructor cnf_constructor(mdd_map, true); // true = lazy encoding
        CNF cnf = cnf_constructor.construct_cnf();

        // Print the variable map for debugging
        const auto& var_map = cnf_constructor.get_variable_map();
        std::cout << "\n========== CNF VARIABLE MAP ==========" << std::endl;
        std::cout << "Variable map size: " << var_map.size() << std::endl;
        if (var_map.empty()) {
            std::cout << "WARNING: Variable map is empty!" << std::endl;
        } else {
            for (const auto& [key, var_id] : var_map) {
                int agent_id = std::get<0>(key);
                auto pos = std::get<1>(key);
                int timestep = std::get<2>(key);
                std::cout << "  " << var_id << ": agent " << agent_id << ", pos (" << pos.first << "," << pos.second << "), t=" << timestep << std::endl;
            }
        }
        std::cout << "========== END CNF VARIABLE MAP ==========" << std::endl;
        std::cout << std::flush;
        
        std::cout << "CNF created: " << cnf.count_variables() << " variables, " 
                  << cnf.count_clauses() << " clauses" << std::endl;
        
        // Print the first 5 clauses and their literals
        const auto& clauses = cnf.get_clauses();
        for (size_t i = 0; i < std::min<size_t>(5, clauses.size()); ++i) {
            std::cerr << "[DEBUG] CNF clause " << (i+1) << ": ";
            for (int lit : clauses[i]) std::cerr << lit << " ";
            std::cerr << std::endl;
        }

        // Step 3: Convert CNF to ProbSAT format
        std::cout << "\n=== Converting CNF to ProbSAT format ===" << std::endl;
        std::vector<int*> clause_pointers;
        std::vector<std::vector<int>> clause_storage;
        cnf_to_probsat_format(cnf, clause_pointers, clause_storage);
        //print the first 5 clauses
        for (size_t i = 0; i < std::min<size_t>(5, clause_storage.size()); ++i) {
            std::cerr << "[C++] clause_storage[" << i << "] = ";
            for (int lit : clause_storage[i]) std::cerr << lit << " ";
            std::cerr << std::endl;
        }
        // Step 4: Solve with ProbSAT
        std::cout << "\n=== Solving with ProbSAT ===" << std::endl;
        
        ProbSatResult result;
        memset(&result, 0, sizeof(result));
        
        // Create an all-0 initial assignment (1-based indexing)
        std::vector<int> initial_assignment(cnf.count_variables() + 1, 0);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        int sat = probsat_solve_in_memory(
            cnf.count_variables(),
            cnf.count_clauses(),
            const_cast<int**>(clause_pointers.data()),
            seed,
            max_runs,
            max_flips,
            &result,
            initial_assignment.data() // All-0 assignment
            //nullptr // No initial assignment
        );
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        // Step 5: Process results
        std::cout << "\n=== Results ===" << std::endl;
        if (sat == 10 || result.sat == 10) {
            std::cout << "SATISFIABLE!" << std::endl;
            std::cout << "Solve time: " << result.solve_time << "s" << std::endl;
            std::cout << "Number of flips: " << result.num_flips << std::endl;
            std::cout << "Total time (including overhead): " << duration.count() << "ms" << std::endl;

            // Print the first clause in the CNF (should be the start clause)
            const auto& clauses = cnf.get_clauses();
            if (!clauses.empty()) {
                std::cout << "First CNF clause: ";
                for (int lit : clauses[0]) std::cout << lit << " ";
                std::cout << std::endl;
            }

            // Print the last clause in the CNF (should be the goal clause)
            if (!clauses.empty()) {
                std::cout << "Last CNF clause: ";
                for (int lit : clauses.back()) std::cout << lit << " ";
                std::cout << std::endl;
            }

            // Print the MDD nodes at the last timestep for the agent
            int last_timestep = 0;
            for (const auto& [level, nodes] : mdd_map[0]->levels) {
                if (level > last_timestep) last_timestep = level;
            }
            std::cout << "Agent 0 MDD nodes at last timestep (" << last_timestep << "): ";
            for (const auto& node : mdd_map[0]->get_nodes_at_level(last_timestep)) {
                std::cout << "(" << node->position.first << "," << node->position.second << ") ";
            }
            std::cout << std::endl;

            // Print the raw ProbSAT assignment
            std::cout << "Raw ProbSAT assignment (first 20 vars): ";
            for (int i = 0; i < std::min(20, cnf.count_variables()); ++i) {
                std::cout << result.assignment[i] << " ";
            }
            std::cout << std::endl;

            std::vector<int> assignment(result.assignment, result.assignment + cnf.count_variables());
            std::cout << "[DEBUG] Assignment vector: ";
            for (int v : assignment) std::cout << v << " ";
            std::cout << std::endl;
            auto agent_paths = cnf_constructor.cnf_assignment_to_paths(assignment);
            std::cout << "[DEBUG] agent_paths result:\n";
            for (const auto& [agent_id, path] : agent_paths) {
                std::cout << "  Agent " << agent_id << ": ";
                for (const auto& pos : path) std::cout << "(" << pos.first << "," << pos.second << ") ";
                std::cout << std::endl;
            }
            
            // Print and visualize the paths
            print_agent_paths(agent_paths);
            
            // Validate paths
            std::cout << "\n=== Path Validation ===" << std::endl;
            for (const auto& [agent_id, path] : agent_paths) {
                bool valid = cnf_constructor.validate_path(agent_id, path);
                std::cout << "Agent " << agent_id << " path is " 
                          << (valid ? "VALID" : "INVALID") << std::endl;
            }
            
        } else {
            std::cout << "UNSATISFIABLE or UNKNOWN" << std::endl;
        }
        
        // Cleanup
        if (result.assignment) {
            free(result.assignment);
        }
        
        std::cout << "\n=== Test completed successfully ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 