#ifndef CNFCONSTRUCTOR_H
#define CNFCONSTRUCTOR_H

#include "CNF.h"
#include "../mdd/MDD.h"
#include <unordered_map>
#include <vector>
#include <memory>
#include <tuple>

// Custom hash function for std::pair and tuples
namespace std {
    template<>
    struct hash<std::pair<int, int>> {
        size_t operator()(const std::pair<int, int>& p) const {
            return hash<int>()(p.first) ^ (hash<int>()(p.second) << 1);
        }
    };
    template<typename T1, typename T2, typename T3>
    struct hash<tuple<T1, T2, T3>> {
        size_t operator()(const tuple<T1, T2, T3>& t) const {
            auto h1 = hash<T1>{}(get<0>(t));
            auto h2 = hash<T2>{}(get<1>(t));
            auto h3 = hash<T3>{}(get<2>(t));
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
}

// CNFConstructor converts Multi-value Decision Diagrams (MDDs) to CNF formulas
// It handles agent path constraints, collision avoidance, and transition constraints
class CNFConstructor {
private:
    std::unordered_map<int, std::shared_ptr<MDD>> mdds; // Map agent_id to MDD
    CNF cnf; // The CNF formula being constructed
    std::unordered_map<std::tuple<int, MDDNode::Position, int>, int> variable_map; // Maps (agent_id, position, timestep) to variable_id
    int next_variable_id; // Next available variable ID
    bool lazy_encoding; // Whether to use lazy encoding for conflicts

public:
    // Constructor: initializes with MDDs and optional parameters
    CNFConstructor(const std::unordered_map<int, std::shared_ptr<MDD>>& mdds, 
                   bool lazy_encoding = false,
                   const CNF& existing_cnf = CNF(),
                   const std::unordered_map<std::tuple<int, MDDNode::Position, int>, int>& existing_variable_map = {},
                   int start_variable_id = 1);

    // Adds a variable for the (agent_id, position, timestep) tuple
    int add_variable(int agent_id, const MDDNode::Position& position, int timestep);

    // Creates clauses ensuring agents follow valid paths in their MDDs
    void create_agent_path_clauses(int agent_id, const std::shared_ptr<MDD>& mdd);

    // Adds clauses ensuring each agent occupies only one position at each timestep
    void add_single_occupancy_clauses();

    // Creates clauses to prevent conflicts between agents (non-lazy encoding)
    void create_no_conflict_clauses();

    // Adds a single collision clause for two agents at a specific position and time
    std::vector<int> add_single_collision_clause(int agent1_id, int agent2_id, 
                                                const MDDNode::Position& position, 
                                                int timestep, bool add_to_cnf = true);

    // Add a single collision clause for two agents at a specific position and time (UNCHECKED - no safety validation)
    std::vector<int> add_single_collision_clause_unchecked(int agent1_id, int agent2_id, 
                                                          const MDDNode::Position& position, 
                                                          int timestep, bool add_to_cnf = true);

    // Adds transition clauses ensuring valid moves between timesteps
    void add_transition_clauses();

    // Main method to construct the complete CNF from MDDs
    CNF construct_cnf(const std::vector<std::tuple<int, int, MDDNode::Position, int>>& collisions = {});
    
    // Add collision clauses to an existing CNF without reconstructing
    void add_collision_clauses_to_cnf(CNF& existing_cnf, const std::vector<std::tuple<int, int, MDDNode::Position, int>>& collisions);
    
    // Add edge collision clauses to an existing CNF without reconstructing
    void add_edge_collision_clauses_to_cnf(CNF& existing_cnf);
    
    // Add a single edge collision clause to an existing CNF
    std::vector<int> add_single_edge_collision_clause_to_cnf(CNF& existing_cnf, int agent1_id, int agent2_id, 
                                                           const MDDNode::Position& pos1, 
                                                           const MDDNode::Position& pos2, 
                                                           int timestep);
    
    // Add a single edge collision clause to an existing CNF (UNCHECKED - no safety validation)
    std::vector<int> add_single_edge_collision_clause_to_cnf_unchecked(CNF& existing_cnf, int agent1_id, int agent2_id, 
                                                                     const MDDNode::Position& pos1, 
                                                                     const MDDNode::Position& pos2, 
                                                                     int timestep);
    
    // Add edge collision clauses to prevent agents from swapping positions
    void add_edge_collision_clauses();
    
    // Add a single edge collision clause for two agents swapping positions (internal use)
    std::vector<int> add_single_edge_collision_clause(int agent1_id, int agent2_id, 
                                                    const MDDNode::Position& pos1, 
                                                    const MDDNode::Position& pos2, 
                                                    int timestep, bool add_to_cnf = true);

    // Add a single edge collision clause for two agents swapping positions (internal use, UNCHECKED - no safety validation)
    std::vector<int> add_single_edge_collision_clause_unchecked(int agent1_id, int agent2_id, 
                                                              const MDDNode::Position& pos1, 
                                                              const MDDNode::Position& pos2, 
                                                              int timestep, bool add_to_cnf = true);

    // Translate a path to CNF variable assignments
    std::vector<int> path_to_cnf_assignment(int agent_id, const std::vector<MDDNode::Position>& path);
    
    // Translate CNF variable assignments to paths for all agents
    std::unordered_map<int, std::vector<MDDNode::Position>> cnf_assignment_to_paths(const std::vector<int>& assignment);
    
    // Validate if a path is consistent with the MDD
    bool validate_path(int agent_id, const std::vector<MDDNode::Position>& path);
    
    // Extract variable ID for a specific (agent, position, timestep)
    int get_variable_id(int agent_id, const MDDNode::Position& position, int timestep) const;

    // Getters for accessing internal state
    const CNF& get_cnf() const { return cnf; }
    const std::unordered_map<std::tuple<int, MDDNode::Position, int>, int>& get_variable_map() const { return variable_map; }
    int get_next_variable_id() const { return next_variable_id; }

private:
    // Helper method to get the maximum number of timesteps across all MDDs
    int get_max_timesteps() const;
};

#endif // CNFCONSTRUCTOR_H 