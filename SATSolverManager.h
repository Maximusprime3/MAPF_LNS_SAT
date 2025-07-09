#ifndef SAT_SOLVER_MANAGER_H
#define SAT_SOLVER_MANAGER_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <filesystem>
#include <unordered_map>
#include <tuple>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <functional>

// Include our MAPF components
#include "mdd/MDD.h"
#include "mdd/MDDNode.h"
#include "mdd/MDDConstructor.h"  // For pair_hash
#include "cnf/CNF.h"
#include "cnf/CNFConstructor.h"
#include "cnf/CNFProbSATConstructor.h"

// Include ProbSAT in-memory API
extern "C" {
    #include "probSAT-master/probSAT_inmem.h"
}

// Forward declarations
class MDD;
class CNF;
class CNFConstructor;

// Struct to hold a row of scenario data from a scenario file
struct ScenarioEntry {
    int bucket;              // Scenario bucket or group identifier
    std::string map_name;    // Name of the map
    int map_width;           // Width of the map
    int map_height;          // Height of the map
    int start_x;             // Agent's start x-coordinate
    int start_y;             // Agent's start y-coordinate
    int goal_x;              // Agent's goal x-coordinate
    int goal_y;              // Agent's goal y-coordinate
    int optimal_length;      // Optimal path length for this agent
};

// Struct to hold ProbSAT solution results
struct ProbSATSolution {
    bool satisfiable;                    // Whether the problem is satisfiable
    std::vector<int> assignment;        // Variable assignments (1-based indexing)
    int num_flips;                      // Number of flips performed
    double solve_time;                  // Time taken to solve (seconds)
    std::string error_message;          // Error message if solving failed
};

// Helper hash function for edges (pairs of positions)
struct edge_hash {
    std::size_t operator()(const std::pair<std::pair<int, int>, std::pair<int, int>>& p) const {
        auto h1 = std::hash<int>{}(p.first.first);
        auto h2 = std::hash<int>{}(p.first.second);
        auto h3 = std::hash<int>{}(p.second.first);
        auto h4 = std::hash<int>{}(p.second.second);
        return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
    }
};

// Type aliases for better readability and semantic meaning
using AgentMDDMap = std::unordered_map<int, std::shared_ptr<MDD>>;  // Maps agent_id -> MDD
using AgentPaths = std::unordered_map<int, std::vector<std::pair<int, int>>>;  // Maps agent_id -> path
using AgentPathsWithPosition = std::unordered_map<int, std::vector<MDDNode::Position>>;  // Maps agent_id -> path with Position type
using VariableMap = std::unordered_map<std::tuple<int, MDDNode::Position, int>, int>;  // Maps (agent_id, position, timestep) -> variable_id
using PositionDistanceMap = std::unordered_map<MDDNode::Position, int, pair_hash>;  // Maps position -> distance from goal
using PositionAgentMap = std::unordered_map<std::pair<int, int>, std::vector<int>, pair_hash>;  // Maps position -> list of agents
using EdgeAgentMap = std::unordered_map<std::pair<std::pair<int, int>, std::pair<int, int>>, std::vector<int>, edge_hash>;  // Maps edge -> list of agents
using NodeMapping = std::unordered_map<const MDDNode*, std::shared_ptr<MDDNode>>;  // Maps old node -> new node

class SATSolverManager {
public:
    // Constructor: Initializes the SATSolverManager with all relevant parameters
    SATSolverManager(const std::string& map_path,
                     const std::string& scenario_path,
                     const std::string& probsat_executable,
                     const std::string& cnf_file_path,
                     int num_agents,
                     int base_max_flips = 1000,
                     int base_max_tries = 100,
                     int seed = 42,
                     const std::string& output_dir = "Data",
                     const std::string& save_name = "",
                     int max_timesteps_threshold = 100,
                     const std::string& time_results_file = "",
                     bool use_flip_heuristic = false,
                     int amount_of_start_goal_sets = -1,
                     const std::string& map_name = "");

    // Loads a map from a file, skipping the first 4 header lines, and returns a 2D grid of chars
    static std::vector<std::vector<char>> load_map(const std::string& map_path);

    // Reads a scenario file and returns a vector of ScenarioEntry structs, one per line (excluding header)
    static std::vector<ScenarioEntry> create_dataframe_from_file(const std::string& file_path);

    // Groups scenario entries into sets of starts and goals for each agent
    // Each set: pair of (vector of starts, vector of goals), where each is a vector of {x, y}
    // The outer vector contains one entry per set (i.e., per group of num_agents)
    static std::vector<std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>>>
    create_starts_and_goals(const std::vector<ScenarioEntry>& entries, int num_agents);

    /**
     * Computes the minimum number of timesteps (makespan) required for all agents to reach their goals.
     * For each agent, computes the shortest path distance from start to goal using MDDConstructor.
     * Returns a pair: (vector of distance matrices, max_timesteps).
     * Each distance matrix is a map from position to distance for that agent.
     * The max_timesteps is the maximum distance from any start to any goal.
     * this is used as the makespan to create the MDDs for all agents.
     */
    static std::pair<std::vector<std::map<std::pair<int, int>, int>>, int>
    compute_max_timesteps(const std::vector<std::vector<char>>& map,
                         const std::vector<std::pair<int, int>>& starts,
                         const std::vector<std::pair<int, int>>& goals);

    /**
     * Creates MDDs for each agent using the map, starts, goals, max_timesteps, and distance matrices.
     * Returns a vector of shared_ptr<MDD>, one for each agent.
     */
    static std::vector<std::shared_ptr<class MDD>>
    create_mdds(const std::vector<std::vector<char>>& map,
                const std::vector<std::pair<int, int>>& starts,
                const std::vector<std::pair<int, int>>& goals,
                int max_timesteps,
                const std::vector<std::map<std::pair<int, int>, int>>& distance_matrices);

    /**
     * Creates a CNF from the MDDs using CNFConstructor. Optionally saves the CNF to a file.
     * @param mdds Vector of shared_ptr<MDD> for each agent.
     * @param save_to_file If true, saves the CNF to a file.
     * @param filename If saving, the filename to use (if empty, auto-generate).
     * @param lazy_encoding If true, use lazy encoding (exclude conflict clauses initially).
     * @return Pair of (shared_ptr<CNF>, filename). Filename is empty if not saved.
     */
    static std::pair<std::shared_ptr<class CNF>, std::string>
    create_and_save_cnf(const std::vector<std::shared_ptr<class MDD>>& mdds,
                       bool save_to_file = false,
                       const std::string& filename = "",
                       bool lazy_encoding = false);

    /**
     * Creates a CNFProbSATConstructor for direct ProbSAT integration.
     * @param mdds Vector of shared_ptr<MDD> for each agent.
     * @param lazy_encoding If true, use lazy encoding (exclude conflict clauses initially).
     * @return Shared pointer to CNFProbSATConstructor.
     */
    static std::shared_ptr<class CNFProbSATConstructor>
    create_cnf_probsat_constructor(const std::vector<std::shared_ptr<class MDD>>& mdds,
                                  bool lazy_encoding = false);

    /**
     * Generates a unique filename by appending a number if the file already exists.
     * @param base_filename The base filename to use.
     * @return A unique filename that does not exist yet.
     */
    static std::string get_unique_filename(const std::string& base_filename);

    /**
     * Converts CNF to ProbSAT format for in-memory solving.
     * @param cnf The CNF formula to convert.
     * @param clause_pointers Output array of pointers to clauses.
     * @param clause_storage Output storage for clause data.
     */
    static void cnf_to_probsat_format(const CNF& cnf, 
                                     std::vector<int*>& clause_pointers, 
                                     std::vector<std::vector<int>>& clause_storage);

    /**
     * Solves a CNF formula using ProbSAT's in-memory API.
     * @param cnf The CNF formula to solve.
     * @param seed Random seed for ProbSAT.
     * @param max_runs Maximum number of runs.
     * @param max_flips Maximum number of flips per run.
     * @param initial_assignment Optional initial assignment (nullptr for random).
     * @return ProbSATSolution containing the results.
     */
    static ProbSATSolution solve_cnf_with_probsat(const CNF& cnf,
                                                 long long seed = 42,
                                                 long long max_runs = 1,
                                                 long long max_flips = 10000,
                                                 const std::vector<int>* initial_assignment = nullptr);

    /**
     * Solves a CNF formula using ProbSAT's in-memory API via CNFProbSATConstructor.
     * @param cnf_constructor The CNFProbSATConstructor to use.
     * @param seed Random seed for ProbSAT.
     * @param max_runs Maximum number of runs.
     * @param max_flips Maximum number of flips per run.
     * @param initial_assignment Optional initial assignment (nullptr for random).
     * @return ProbSATSolution containing the results.
     */
    static ProbSATSolution solve_cnf_with_probsat(const std::shared_ptr<class CNFProbSATConstructor>& cnf_constructor,
                                                 long long seed = 42,
                                                 long long max_runs = 1,
                                                 long long max_flips = 10000,
                                                 const std::vector<int>* initial_assignment = nullptr);

    /**
     * Extracts agent paths from a ProbSAT solution using CNFConstructor.
     * @param cnf_constructor The CNFConstructor used to create the CNF.
     * @param assignment The variable assignment from ProbSAT.
     * @return Map from agent_id to path (vector of positions).
     */
    static AgentPaths 
    extract_agent_paths_from_solution(CNFConstructor& cnf_constructor,
                                     const std::vector<int>& assignment);

    /**
     * Validates agent paths against their MDDs.
     * @param cnf_constructor The CNFConstructor used to create the CNF.
     * @param agent_paths Map from agent_id to path.
     * @return True if all paths are valid, false otherwise.
     */
    static bool validate_agent_paths(CNFConstructor& cnf_constructor,
                                   const AgentPaths& agent_paths);

    /**
     * Prints agent paths in a readable format.
     * @param agent_paths Map from agent_id to path.
     */
    static void print_agent_paths(const AgentPaths& agent_paths);

    /**
     * Calculates max flips and tries based on CNF size (heuristic).
     * @param cnf The CNF formula.
     * @param base_max_flips Base number of max flips.
     * @param base_max_tries Base number of max tries.
     * @return Pair of (max_flips, max_tries).
     */
    static std::pair<long long, long long> calculate_max_flips_and_tries(const CNF& cnf,
                                                                        long long base_max_flips = 1000,
                                                                        long long base_max_tries = 100);

    /**
     * Detects vertex collisions (two agents at same position at same time).
     * @param agent_paths Map from agent_id to path (vector of positions).
     * @return Vector of collision tuples (agent1_id, agent2_id, position, timestep).
     */
    static std::vector<std::tuple<int, int, std::pair<int, int>, int>> 
    find_vertex_collisions(const AgentPaths& agent_paths);

    /**
     * Detects edge collisions (agents swapping positions between consecutive timesteps).
     * @param agent_paths Map from agent_id to path (vector of positions).
     * @return Vector of edge collision tuples (agent1_id, agent2_id, pos1, pos2, timestep).
     */
    static std::vector<std::tuple<int, int, std::pair<int, int>, std::pair<int, int>, int>> 
    find_edge_collisions(const AgentPaths& agent_paths);

    /**
     * Detects all collisions (both vertex and edge collisions).
     * @param agent_paths Map from agent_id to path (vector of positions).
     * @return Pair of (vertex_collisions, edge_collisions).
     */
    static std::pair<std::vector<std::tuple<int, int, std::pair<int, int>, int>>,
                     std::vector<std::tuple<int, int, std::pair<int, int>, std::pair<int, int>, int>>> 
    find_all_collisions(const AgentPaths& agent_paths);

    /**
     * Prints collision information in a readable format.
     * @param vertex_collisions Vector of vertex collision tuples.
     * @param edge_collisions Vector of edge collision tuples.
     */
    static void print_collisions(const std::vector<std::tuple<int, int, std::pair<int, int>, int>>& vertex_collisions,
                                const std::vector<std::tuple<int, int, std::pair<int, int>, std::pair<int, int>, int>>& edge_collisions);

    /**
     * Generates clauses to prevent edge collisions and adds them to the CNF.
     * @param cnf_constructor The CNFProbSATConstructor to add clauses to.
     * @param edge_collisions Vector of edge collision tuples.
     * @return Number of clauses added.
     */
    static int add_edge_collision_prevention_clauses(std::shared_ptr<CNFProbSATConstructor>& cnf_constructor,
                                                    const std::vector<std::tuple<int, int, std::pair<int, int>, std::pair<int, int>, int>>& edge_collisions);

    /**
     * Generates clauses to prevent vertex collisions and adds them to the CNF.
     * @param cnf_constructor The CNFProbSATConstructor to add clauses to.
     * @param vertex_collisions Vector of vertex collision tuples.
     * @return Number of clauses added.
     */
    static int add_vertex_collision_prevention_clauses(std::shared_ptr<CNFProbSATConstructor>& cnf_constructor,
                                                      const std::vector<std::tuple<int, int, std::pair<int, int>, int>>& vertex_collisions);

    // ... (other public methods will be added later)

protected:
    // Member variables corresponding to Python __init__
    std::vector<std::vector<char>> map; // The map grid
    std::string map_name;               // Name of the map
    std::vector<int> map_size;          // Size of the map [rows, cols]
    std::vector<std::pair<std::vector<int>, std::vector<int>>> starts_and_goals; // Sets of starts and goals
    std::string probsat_executable;     // Path to the probSAT executable
    std::string cnf_file_path;          // Path to the CNF file
    int num_agents;                     // Number of agents
    int base_max_flips;                 // Base number of maximum flips for probSAT
    int base_max_tries;                 // Base number of maximum tries for probSAT
    int seed;                           // Random seed
    std::string output_dir;             // Output directory
    std::string save_name;              // Save name for results
    int max_timesteps_threshold;        // Maximum number of timesteps
    std::string time_results_file;      // File to save timing results
    bool use_flip_heuristic;            // Whether to use heuristic-based max flips/tries
    int amount_of_start_goal_sets;      // Number of start/goal sets to use (-1 for all)
    std::map<std::string, std::string> parameters; // Parameters for reference
    std::map<std::string, std::string> cnf_cache;  // Cache for CNF files
    // Add more as needed for later methods
};

#endif // SAT_SOLVER_MANAGER_H 