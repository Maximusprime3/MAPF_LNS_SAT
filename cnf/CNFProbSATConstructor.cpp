#include "CNFProbSATConstructor.h"
#include <algorithm>
#include <iostream>

CNFProbSATConstructor::CNFProbSATConstructor(const std::unordered_map<int, std::shared_ptr<MDD>>& mdds, 
                                             bool lazy_encoding,
                                             const std::unordered_map<std::tuple<int, MDDNode::Position, int>, int>& existing_variable_map,
                                             int start_variable_id)
    : CNFConstructor(mdds, lazy_encoding, CNF(), existing_variable_map, start_variable_id) {
    // Initialize ProbSAT-specific storage
    probsat_clause_storage.clear();
    probsat_clause_pointers.clear();
}

void CNFProbSATConstructor::add_clause(const std::vector<int>& clause) {
    // Store clause in ProbSAT format (null-terminated)
    std::vector<int> clause_copy = clause;
    clause_copy.push_back(0); // null-terminate for ProbSAT
    probsat_clause_storage.push_back(clause_copy);
}

void CNFProbSATConstructor::construct_probsat_cnf(const std::vector<std::tuple<int, int, MDDNode::Position, int>>& collisions) {
    // Clear any existing clauses
    probsat_clause_storage.clear();
    
    // Manually construct clauses in ProbSAT format
    // Agent path clauses
    for (const auto& agent_mdd_pair : mdds) {
        int agent_id = agent_mdd_pair.first;
        const auto& mdd = agent_mdd_pair.second;
        
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
    
    // Single occupancy clauses
    for (const auto& agent_mdd_pair : mdds) {
        int agent_id = agent_mdd_pair.first;
        const auto& mdd = agent_mdd_pair.second;
        for (int timestep = 0; timestep <= get_max_timesteps(); ++timestep) {
            const auto& nodes_at_timestep = mdd->get_nodes_at_level(timestep);
            if (nodes_at_timestep.size() > 1) {
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
    
    // No conflict clauses (if not lazy encoding)
    if (!lazy_encoding) {
        std::vector<int> all_agent_ids;
        for (const auto& agent_mdd_pair : mdds) {
            all_agent_ids.push_back(agent_mdd_pair.first);
        }
        for (int timestep = 0; timestep <= get_max_timesteps(); ++timestep) {
            for (size_t i = 0; i < all_agent_ids.size(); ++i) {
                for (size_t j = i + 1; j < all_agent_ids.size(); ++j) {
                    int agent1_id = all_agent_ids[i];
                    int agent2_id = all_agent_ids[j];
                    const auto& agent1_nodes = mdds.at(agent1_id)->get_nodes_at_level(timestep);
                    const auto& agent2_nodes = mdds.at(agent2_id)->get_nodes_at_level(timestep);
                    for (const auto& node1 : agent1_nodes) {
                        for (const auto& node2 : agent2_nodes) {
                            if (node1->position == node2->position) {
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
        
        // Edge collision clauses
        add_edge_collision_clauses();
    }
    
    // Transition clauses
    for (const auto& agent_mdd_pair : mdds) {
        int agent_id = agent_mdd_pair.first;
        const auto& mdd = agent_mdd_pair.second;
        
        std::vector<std::pair<int, std::vector<std::shared_ptr<MDDNode>>>> sorted_levels;
        for (const auto& level_pair : mdd->levels) {
            sorted_levels.push_back(level_pair);
        }
        std::sort(sorted_levels.begin(), sorted_levels.end());
        
        for (const auto& level_pair : sorted_levels) {
            int t = level_pair.first;
            if (mdd->levels.find(t + 1) == mdd->levels.end()) {
                continue; // No next timestep
            }
            
            for (const auto& node : level_pair.second) {
                if (!node->children.empty()) {
                    std::vector<int> clause = {-add_variable(agent_id, node->position, t)};
                    for (const auto& child : node->children) {
                        clause.push_back(add_variable(agent_id, child->position, t + 1));
                    }
                    add_clause(clause);
                }
            }
        }
    }
    
    // Add specific collision clauses if provided
    for (const auto& collision : collisions) {
        int agent1_id, agent2_id, timestep;
        MDDNode::Position position;
        std::tie(agent1_id, agent2_id, position, timestep) = collision;
        add_single_collision_clause(agent1_id, agent2_id, position, timestep);
    }
    
    // Build clause pointers for ProbSAT
    build_clause_pointers();
}

void CNFProbSATConstructor::build_clause_pointers() {
    probsat_clause_pointers.clear();
    probsat_clause_pointers.push_back(nullptr); // 1-based indexing for ProbSAT
    for (auto& clause_vec : probsat_clause_storage) {
        probsat_clause_pointers.push_back(clause_vec.data());
    }
}

int** CNFProbSATConstructor::get_probsat_clause_pointers() {
    return probsat_clause_pointers.empty() ? nullptr : probsat_clause_pointers.data();
}

int CNFProbSATConstructor::get_probsat_num_clauses() const {
    return probsat_clause_pointers.empty() ? 0 : (int)probsat_clause_pointers.size() - 1;
}

int CNFProbSATConstructor::get_probsat_num_variables() const {
    return get_next_variable_id() - 1;
} 