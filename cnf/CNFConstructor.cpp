#include "CNFConstructor.h"
#include <algorithm>
#include <iostream>

CNFConstructor::CNFConstructor(const std::unordered_map<int, std::shared_ptr<MDD>>& mdds, 
                               bool lazy_encoding,
                               const CNF& existing_cnf,
                               const std::unordered_map<std::tuple<int, MDDNode::Position, int>, int>& existing_variable_map,
                               int start_variable_id)
    : mdds(mdds), cnf(), variable_map(existing_variable_map), next_variable_id(start_variable_id), lazy_encoding(lazy_encoding) {
    // Copy existing CNF clauses
    for (const auto& clause : existing_cnf.get_clauses()) {
        cnf.add_clause(clause);
    }
}

int CNFConstructor::add_variable(int agent_id, const MDDNode::Position& position, int timestep) {
    auto key = std::make_tuple(agent_id, position, timestep);
    if (variable_map.find(key) == variable_map.end()) {
        variable_map[key] = next_variable_id;
        next_variable_id++;
    }
    return variable_map[key];
}

void CNFConstructor::create_agent_path_clauses(int agent_id, const std::shared_ptr<MDD>& mdd) {
    // For each level in the MDD, create clauses to ensure valid paths
    // Sort levels by timestep to ensure consistent variable creation order
    std::vector<std::pair<int, std::vector<std::shared_ptr<MDDNode>>>> sorted_levels;
    for (const auto& level_pair : mdd->levels) {
        sorted_levels.push_back(level_pair);
    }
    std::sort(sorted_levels.begin(), sorted_levels.end());
    for (const auto& level_pair : sorted_levels) {
        int timestep = level_pair.first;
        const auto& nodes = level_pair.second;
        
        // Ensure the agent is at one of the possible nodes at this timestep
        std::vector<int> valid_nodes_clause;
        for (const auto& node : nodes) {
            valid_nodes_clause.push_back(add_variable(agent_id, node->position, timestep));
        }
        add_clause(valid_nodes_clause);
    }
}

void CNFConstructor::add_clause(const std::vector<int>& clause) {
    cnf.add_clause(clause);
}

void CNFConstructor::add_single_occupancy_clauses() {
    // Add clauses to ensure each agent occupies only one position at each timestep
    for (const auto& agent_mdd_pair : mdds) {
        int agent_id = agent_mdd_pair.first;
        const auto& mdd = agent_mdd_pair.second;
        for (int timestep = 0; timestep <= get_max_timesteps(); ++timestep) {
            auto nodes_at_timestep = mdd->get_nodes_at_level(timestep);
            if (nodes_at_timestep.size() > 1) {
                // Create clauses that ensure only one of these nodes can be true at a time
                for (size_t i = 0; i < nodes_at_timestep.size(); ++i) {
                    for (size_t j = i + 1; j < nodes_at_timestep.size(); ++j) {
                        std::vector<int> clause = {
                            -add_variable(agent_id, nodes_at_timestep[i]->position, timestep),
                            -add_variable(agent_id, nodes_at_timestep[j]->position, timestep)
                        };
                        add_clause(clause);
                    }
                }
            }
        }
    }
}

// for eager encoding create all no conflict clauses
void CNFConstructor::create_no_conflict_clauses() {
    // Ensure no two agents are at the same node at the same timestep
    std::vector<int> all_agent_ids;
    for (const auto& agent_mdd_pair : mdds) {
        all_agent_ids.push_back(agent_mdd_pair.first);
    }
    for (int timestep = 0; timestep <= get_max_timesteps(); ++timestep) {
        // Generate all pairs of agents
        for (size_t i = 0; i < all_agent_ids.size(); ++i) {
            for (size_t j = i + 1; j < all_agent_ids.size(); ++j) {
                int agent1_id = all_agent_ids[i];
                int agent2_id = all_agent_ids[j];
                
                // Check which nodes they can be at in this timestep
                auto agent1_nodes = mdds.at(agent1_id)->get_nodes_at_level(timestep);
                auto agent2_nodes = mdds.at(agent2_id)->get_nodes_at_level(timestep);
                
                // Add clauses to ensure they don't occupy the same node
                for (const auto& node1 : agent1_nodes) {
                    for (const auto& node2 : agent2_nodes) {
                        if (node1->position == node2->position) {
                            // The negation ensures they don't both occupy the same node
                            std::vector<int> conflict_clause = {
                                -add_variable(agent1_id, node1->position, timestep),
                                -add_variable(agent2_id, node2->position, timestep)
                            };
                            add_clause(conflict_clause);
                        }
                    }
                }
            }
        }
    }
}

std::vector<int> CNFConstructor::add_single_collision_clause(int agent1_id, int agent2_id, 
                                                            const MDDNode::Position& position, 
                                                            int timestep, bool add_to_cnf) {
    int variable1 = add_variable(agent1_id, position, timestep);
    int variable2 = add_variable(agent2_id, position, timestep);
    
    // Negate the variables to ensure only one can occupy this position at this time
    std::vector<int> clause = {-variable1, -variable2};
    if (add_to_cnf) {
        add_clause(clause);
    }
    return clause;
}

void CNFConstructor::add_transition_clauses() {
    // For every agent and every timestep (except the last), enforce that if the agent is at a given node u at time t,
    // then at time t+1 it must be at one of the children of u as defined in the MDD.
    for (const auto& agent_mdd_pair : mdds) {
        int agent_id = agent_mdd_pair.first;
        const auto& mdd = agent_mdd_pair.second;
        
        // Sort levels by timestep to ensure consistent processing order
        std::vector<std::pair<int, std::vector<std::shared_ptr<MDDNode>>>> sorted_levels;
        for (const auto& level_pair : mdd->levels) {
            sorted_levels.push_back(level_pair);
        }
        std::sort(sorted_levels.begin(), sorted_levels.end());
        
        // For each timestep that has a subsequent timestep:
        for (const auto& level_pair : sorted_levels) {
            int t = level_pair.first;
            if (mdd->levels.find(t + 1) == mdd->levels.end()) {
                continue; // No next timestep
            }
            
            // For every node at time t:
            for (const auto& node : level_pair.second) {
                if (!node->children.empty()) { // Only add if there is at least one valid move
                    // Build a clause: ¬x(agent, u, t) ∨ (x(agent, child1, t+1) ∨ ... ∨ x(agent, child_k, t+1))
                    std::vector<int> clause = {-add_variable(agent_id, node->position, t)};
                    for (const auto& child : node->children) {
                        clause.push_back(add_variable(agent_id, child->position, t + 1));
                    }
                    add_clause(clause);
                }
                // If no children exist, the MDD would be incomplete—typically waiting is allowed.
            }
        }
    }
}

CNF CNFConstructor::construct_cnf(const std::vector<std::tuple<int, int, MDDNode::Position, int>>& collisions) {
    // Construct CNF from the given MDDs
    for (const auto& agent_mdd_pair : mdds) {
        create_agent_path_clauses(agent_mdd_pair.first, agent_mdd_pair.second);
    }
    add_single_occupancy_clauses();
    if (!lazy_encoding) {
        create_no_conflict_clauses();
        // Add edge collision clauses to prevent position swapping (eager encoding only)
        add_edge_collision_clauses();
    }
    add_transition_clauses();
    
    // Add specific collision clauses if provided
    for (const auto& collision : collisions) {
        int agent1_id, agent2_id, timestep;
        MDDNode::Position position;
        std::tie(agent1_id, agent2_id, position, timestep) = collision;
        add_single_collision_clause(agent1_id, agent2_id, position, timestep);
    }

    return cnf;
}

void CNFConstructor::add_collision_clauses_to_cnf(CNF& existing_cnf, const std::vector<std::tuple<int, int, MDDNode::Position, int>>& collisions) {
    // Add specific collision clauses to the existing CNF
    for (const auto& collision : collisions) {
        int agent1_id, agent2_id, timestep;
        MDDNode::Position position;
        std::tie(agent1_id, agent2_id, position, timestep) = collision;
        
        // Create the collision clause
        std::vector<int> clause = add_single_collision_clause(agent1_id, agent2_id, position, timestep, false);
        
        // Add it to the existing CNF
        existing_cnf.add_clause(clause);
    }
}



std::vector<int> CNFConstructor::add_single_edge_collision_clause(int agent1_id, int agent2_id, 
                                                                const MDDNode::Position& pos1, 
                                                                const MDDNode::Position& pos2, 
                                                                int timestep, bool add_to_cnf) {
    int var1 = get_variable_id(agent1_id, pos1, timestep);
    int var2 = get_variable_id(agent1_id, pos2, timestep + 1);
    int var3 = get_variable_id(agent2_id, pos2, timestep);
    int var4 = get_variable_id(agent2_id, pos1, timestep + 1);
    if (var1 > 0 && var2 > 0 && var3 > 0 && var4 > 0) {
        std::vector<int> clause = {-var1, -var2, -var3, -var4};
        if (add_to_cnf) {
            cnf.add_clause(clause);
        }
        return clause;
    }
    throw std::runtime_error("Invalid edge collision clause: agent1=" + std::to_string(agent1_id) + ", agent2=" + std::to_string(agent2_id) + ", pos1=(" + std::to_string(pos1.first) + "," + std::to_string(pos1.second) + "), pos2=(" + std::to_string(pos2.first) + "," + std::to_string(pos2.second) + "), timestep=" + std::to_string(timestep));
}



// for eager encoding create all edge collision clauses
void CNFConstructor::add_edge_collision_clauses() {
    // Prevent agents from swapping positions by traversing the same edge in opposite directions
    std::vector<int> all_agent_ids;
    for (const auto& agent_mdd_pair : mdds) {
        all_agent_ids.push_back(agent_mdd_pair.first);
    }
    
    for (int timestep = 0; timestep < get_max_timesteps(); ++timestep) {
        // Generate all pairs of agents
        for (size_t i = 0; i < all_agent_ids.size(); ++i) {
            for (size_t j = i + 1; j < all_agent_ids.size(); ++j) {
                int agent1_id = all_agent_ids[i];
                int agent2_id = all_agent_ids[j];
                
                // Get nodes at current and next timestep for both agents
                auto agent1_nodes_t = mdds.at(agent1_id)->get_nodes_at_level(timestep);
                auto agent1_nodes_t1 = mdds.at(agent1_id)->get_nodes_at_level(timestep + 1);
                auto agent2_nodes_t = mdds.at(agent2_id)->get_nodes_at_level(timestep);
                auto agent2_nodes_t1 = mdds.at(agent2_id)->get_nodes_at_level(timestep + 1);
                
                // Check for edge conflicts: agent1 goes from pos1 to pos2 while agent2 goes from pos2 to pos1
                for (const auto& node1_t : agent1_nodes_t) {
                    for (const auto& node1_t1 : agent1_nodes_t1) {
                        // Check if this is a valid transition for agent1
                        bool valid_transition1 = false;
                        for (const auto& child : node1_t->children) {
                            if (child->position == node1_t1->position) {
                                valid_transition1 = true;
                                break;
                            }
                        }
                        if (!valid_transition1) continue;
                        
                        for (const auto& node2_t : agent2_nodes_t) {
                            for (const auto& node2_t1 : agent2_nodes_t1) {
                                // Check if this is a valid transition for agent2
                                bool valid_transition2 = false;
                                for (const auto& child : node2_t->children) {
                                    if (child->position == node2_t1->position) {
                                        valid_transition2 = true;
                                        break;
                                    }
                                }
                                if (!valid_transition2) continue;
                                
                                // Check for edge collision: agent1 goes from pos1 to pos2, agent2 goes from pos2 to pos1
                                if (node1_t->position == node2_t1->position && node1_t1->position == node2_t->position) {
                                    add_single_edge_collision_clause(agent1_id, agent2_id, 
                                                                  node1_t->position, node1_t1->position, timestep);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}



std::vector<int> CNFConstructor::path_to_cnf_assignment(int agent_id, const std::vector<MDDNode::Position>& path) {
    std::vector<int> assignment;
    
    // For each timestep, set the variable for the agent's position to true
    for (size_t timestep = 0; timestep < path.size(); ++timestep) {
        int variable_id = get_variable_id(agent_id, path[timestep], timestep);
        if (variable_id > 0) {
            assignment.push_back(variable_id);
        }
    }
    
    return assignment;
}

std::unordered_map<int, std::vector<MDDNode::Position>> CNFConstructor::cnf_assignment_to_paths(const std::vector<int>& assignment) {
    std::unordered_map<int, std::vector<MDDNode::Position>> agent_paths;
    
    std::cout << "\n=== DEBUG: cnf_assignment_to_paths ===" << std::endl;
    std::cout << "Input assignment: ";
    for (int v : assignment) std::cout << v << " ";
    std::cout << std::endl;
    
    std::cout << "Variable map contents:" << std::endl;
    for (const auto& var_pair : variable_map) {
        int agent_id = std::get<0>(var_pair.first);
        auto pos = std::get<1>(var_pair.first);
        int timestep = std::get<2>(var_pair.first);
        int var_id = var_pair.second;
        std::cout << "  Var " << var_id << ": agent " << agent_id 
                  << ", pos (" << pos.first << "," << pos.second << "), t=" << timestep << std::endl;
    }
    
    // Group positive assignments by agent
    std::unordered_map<int, std::vector<std::pair<int, MDDNode::Position>>> agent_positions;
    
    // The assignment contains truth values (1 for true, 0 for false) for each variable
    // We need to iterate through the assignment by variable index
    for (size_t i = 0; i < assignment.size(); ++i) {
        int var_id = i + 1; // ProbSAT uses 1-based indexing
        int truth_value = assignment[i];
        
        if (truth_value > 0) { // Only positive assignments
            std::cout << "Processing variable " << var_id << " (truth value: " << truth_value << ")" << std::endl;
            
            // Find the (agent, position, timestep) for this variable
            for (const auto& var_pair : variable_map) {
                if (var_pair.second == var_id) {
                    int agent_id = std::get<0>(var_pair.first);
                    MDDNode::Position position = std::get<1>(var_pair.first);
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
        std::vector<std::pair<int, MDDNode::Position>> sorted_positions = positions;
        std::sort(sorted_positions.begin(), sorted_positions.end());
        
        std::cout << "Agent " << agent_id << " positions after sorting:" << std::endl;
        for (const auto& pos_pair : sorted_positions) {
            std::cout << "  t=" << pos_pair.first << ": (" << pos_pair.second.first 
                      << "," << pos_pair.second.second << ")" << std::endl;
        }
        
        // Extract path
        std::vector<MDDNode::Position> path;
        for (const auto& pos_pair : sorted_positions) {
            path.push_back(pos_pair.second);
        }
        
        std::cout << "Final path: ";
        for (const auto& pos : path) {
            std::cout << "(" << pos.first << "," << pos.second << ") ";
        }
        std::cout << std::endl;
        
        agent_paths[agent_id] = path;
    }
    
    std::cout << "=== END DEBUG ===" << std::endl;
    
    return agent_paths;
}

bool CNFConstructor::validate_path(int agent_id, const std::vector<MDDNode::Position>& path) {
    if (path.empty()) return false;
    
    // Check if agent exists
    if (mdds.find(agent_id) == mdds.end()) return false;
    
    const auto& mdd = mdds.at(agent_id);
    
    // Check each position is valid at its timestep
    for (size_t timestep = 0; timestep < path.size(); ++timestep) {
        auto nodes_at_timestep = mdd->get_nodes_at_level(timestep);
        bool position_valid = false;
        
        for (const auto& node : nodes_at_timestep) {
            if (node->position == path[timestep]) {
                position_valid = true;
                break;
            }
        }
        
        if (!position_valid) {
            return false;
        }
    }
    
    // Check transitions are valid (optional - could be more strict)
    for (size_t timestep = 0; timestep < path.size() - 1; ++timestep) {
        // Find the node at current timestep
        auto nodes_at_timestep = mdd->get_nodes_at_level(timestep);
        std::shared_ptr<MDDNode> current_node = nullptr;
        
        for (const auto& node : nodes_at_timestep) {
            if (node->position == path[timestep]) {
                current_node = node;
                break;
            }
        }
        
        if (current_node) {
            // Check if next position is a valid child
            bool valid_transition = false;
            for (const auto& child : current_node->children) {
                if (child->position == path[timestep + 1]) {
                    valid_transition = true;
                    break;
                }
            }
            
            if (!valid_transition) {
                return false;
            }
        }
    }
    
    return true;
}

int CNFConstructor::get_variable_id(int agent_id, const MDDNode::Position& position, int timestep) const {
    auto key = std::make_tuple(agent_id, position, timestep);
    auto it = variable_map.find(key);
    if (it != variable_map.end()) {
        return it->second;
    }
    return -1; // Variable not found
}

int CNFConstructor::get_max_timesteps() const {
    int max_timesteps = 0;
    for (const auto& agent_mdd_pair : mdds) {
        for (const auto& level_pair : agent_mdd_pair.second->levels) {
            max_timesteps = std::max(max_timesteps, level_pair.first);
        }
    }
    return max_timesteps;
} 

// Returns a vector of positive literals for the given agent paths (for use as assumptions in a CDCL solver).
std::vector<int> CNFConstructor::partial_assignment_from_paths(const std::unordered_map<int, std::vector<MDDNode::Position>>& agent_paths) const {
    std::vector<int> assumptions;
    // For each agent in the provided map
    for (const auto& agent_path_pair : agent_paths) {
        int agent_id = agent_path_pair.first;
        const auto& path = agent_path_pair.second;
        // Skip agents with empty paths (e.g., those involved in collisions)
        if (path.empty()) continue;
        // For each timestep, add the variable for (agent, position, timestep)
        for (size_t timestep = 0; timestep < path.size(); ++timestep) {
            int var_id = get_variable_id(agent_id, path[timestep], timestep);
            if (var_id > 0) {
                assumptions.push_back(var_id); // Positive literal: set this variable to true
            }
        }
    }
    return assumptions;
} 