#include "SATSolverManager.h"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cctype>
#include "mdd/MDDConstructor.h"
#include "mdd/MDDNode.h"
#include <map>
#include "mdd/MDD.h"
#include "cnf/CNFConstructor.h"
#include "cnf/CNF.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <sys/stat.h> // for checking file existence

SATSolverManager::SATSolverManager(const std::string& map_path,
                                   const std::string& scenario_path,
                                   const std::string& probsat_executable,
                                   const std::string& cnf_file_path,
                                   int num_agents,
                                   int base_max_flips,
                                   int base_max_tries,
                                   int seed,
                                   const std::string& output_dir,
                                   const std::string& save_name,
                                   int max_timesteps_threshold,
                                   const std::string& time_results_file,
                                   bool use_flip_heuristic,
                                   int amount_of_start_goal_sets,
                                   const std::string& map_name)
    : map_name(map_name),
      probsat_executable(probsat_executable),
      cnf_file_path(cnf_file_path),
      num_agents(num_agents),
      base_max_flips(base_max_flips),
      base_max_tries(base_max_tries),
      seed(seed),
      output_dir(output_dir),
      save_name(save_name),
      max_timesteps_threshold(max_timesteps_threshold),
      time_results_file(time_results_file),
      use_flip_heuristic(use_flip_heuristic),
      amount_of_start_goal_sets(amount_of_start_goal_sets)
{
    // TODO: Implement map loading from map_path
    // map = load_map(map_path);
    // map_size = {map.size(), map.empty() ? 0 : map[0].size()};

    // TODO: Implement starts_and_goals loading from scenario_path
    // starts_and_goals = load_starts_and_goals_from_file(scenario_path, num_agents);
    // if (amount_of_start_goal_sets > 0 && starts_and_goals.size() > amount_of_start_goal_sets)
    //     starts_and_goals.resize(amount_of_start_goal_sets);

    // Create output directory if it doesn't exist
    std::filesystem::create_directories(output_dir);

    // Fill parameters map (for reference, not strictly needed in C++)
    parameters["map_path"] = map_path;
    parameters["scenario_path"] = scenario_path;
    parameters["probsat_executable"] = probsat_executable;
    parameters["cnf_file_path"] = cnf_file_path;
    parameters["num_agents"] = std::to_string(num_agents);
    parameters["base_max_flips"] = std::to_string(base_max_flips);
    parameters["base_max_tries"] = std::to_string(base_max_tries);
    parameters["seed"] = std::to_string(seed);
    parameters["output_dir"] = output_dir;
    parameters["save_name"] = save_name;
    parameters["max_timesteps_threshold"] = std::to_string(max_timesteps_threshold);
    parameters["time_results_file"] = time_results_file;
    parameters["use_flip_heuristic"] = use_flip_heuristic ? "true" : "false";
    parameters["amount_of_start_goal_sets"] = std::to_string(amount_of_start_goal_sets);
    parameters["map_name"] = map_name;

    // cnf_cache is empty initially
}

std::vector<std::vector<char>> SATSolverManager::load_map(const std::string& map_path) {
    std::ifstream infile(map_path);
    std::vector<std::vector<char>> map;
    if (!infile) {
        std::cerr << "Error: Could not open map file: " << map_path << std::endl;
        return map;
    }

    std::string line;
    // Skip the first 4 lines (header)
    for (int i = 0; i < 4 && std::getline(infile, line); ++i) {}

    bool has_swamp_or_water = false;
    // Read the rest of the file as the map
    while (std::getline(infile, line)) {
        std::vector<char> row;
        for (char c : line) {
            if (c != '\r' && c != '\n') {
                row.push_back(c);
                if (c == 'S' || c == 'W') has_swamp_or_water = true;
            }
        }
        if (!row.empty()) map.push_back(row);
    }
    if (has_swamp_or_water) {
        std::cerr << "Warning: Swamp (S) and Water (W) will be handled as unpassable terrain" << std::endl;
    }
    return map;
}

// Reads a scenario file and returns a vector of ScenarioEntry structs, one per line (excluding header)
std::vector<ScenarioEntry> SATSolverManager::create_dataframe_from_file(const std::string& file_path) {
    std::ifstream infile(file_path);
    std::vector<ScenarioEntry> entries;
    if (!infile) {
        std::cerr << "Error: Could not open scenario file: " << file_path << std::endl;
        return entries;
    }

    std::string line;
    bool first_line = true;
    while (std::getline(infile, line)) {
        // Skip the first line if it starts with 'version'
        if (first_line) {
            if (line.rfind("version", 0) == 0) {
                first_line = false;
                continue; // skip version line
            }
            first_line = false;
        }
        if (line.empty()) continue;
        std::istringstream iss(line);
        std::vector<std::string> tokens;
        std::string token;
        // Split the line by tab character into columns
        while (std::getline(iss, token, '\t')) {
            tokens.push_back(token);
        }
        // Each line should have exactly 9 columns
        if (tokens.size() != 9) {
            std::cerr << "Warning: Malformed scenario line (expected 9 columns): " << line << std::endl;
            continue;
        }
        ScenarioEntry entry;
        try {
            // Convert numeric fields from string to int, assign map_name as string
            entry.bucket = std::stoi(tokens[0]);
            entry.map_name = tokens[1];
            entry.map_width = std::stoi(tokens[2]);
            entry.map_height = std::stoi(tokens[3]);
            entry.start_x = std::stoi(tokens[4]);
            entry.start_y = std::stoi(tokens[5]);
            entry.goal_x = std::stoi(tokens[6]);
            entry.goal_y = std::stoi(tokens[7]);
            entry.optimal_length = std::stoi(tokens[8]);
        } catch (const std::exception& e) {
            std::cerr << "Warning: Failed to parse scenario line: " << line << "\n" << e.what() << std::endl;
            continue;
        }
        entries.push_back(entry);
    }
    return entries;
}

// Groups scenario entries into sets of starts and goals for each agent
// Each set: pair of (vector of starts, vector of goals), where each is a vector of {x, y}
// The outer vector contains one entry per set (i.e., per group of num_agents)
std::vector<std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>>>
SATSolverManager::create_starts_and_goals(const std::vector<ScenarioEntry>& entries, int num_agents) {
    std::vector<std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>>> sets;
    // Calculate the number of sets (groups of num_agents)
    int num_sets = entries.size() / num_agents + (entries.size() % num_agents == 0 ? 0 : 1);
    for (int i = 0; i < num_sets; ++i) {
        std::vector<std::pair<int, int>> starts, goals;
        // For each agent in the set, extract start and goal positions
        for (int j = 0; j < num_agents; ++j) {
            int idx = i * num_agents + j;
            if (idx >= entries.size()) break;
            starts.emplace_back(entries[idx].start_x, entries[idx].start_y);
            goals.emplace_back(entries[idx].goal_x, entries[idx].goal_y);
        }
        // Only add non-empty sets
        if (!starts.empty() && !goals.empty()) {
            sets.emplace_back(starts, goals);
        }
    }
    return sets;
}

/**
 * Computes the maximum number of timesteps (makespan) required for all agents to reach their goals.
 * For each agent, computes the shortest path distance from start to goal using MDDConstructor.
 * Returns a pair: (vector of distance matrices, max_timesteps).
 * Each distance matrix is a map from position to the goal for that agent.
 * The max_timesteps is the maximum distance from any start to any goal.
 * this is used as the makespan to create the MDDs for all agents.
 */
std::pair<std::vector<std::map<std::pair<int, int>, int>>, int>
SATSolverManager::compute_max_timesteps(
    const std::vector<std::vector<char>>& map,
    const std::vector<std::pair<int, int>>& starts,
    const std::vector<std::pair<int, int>>& goals)
{
    std::vector<std::map<std::pair<int, int>, int>> distance_matrices;
    std::vector<int> goal_distances;

    // For each agent, compute the distance matrix and the distance from start to goal
    for (size_t i = 0; i < starts.size(); ++i) {
        MDDConstructor constructor(map, starts[i], goals[i]);
        // Compute all distances from every position to the goal
        auto distances = constructor.compute_all_distances();
        // Convert unordered_map to std::map for return type compatibility
        std::map<std::pair<int, int>, int> dist_map;
        for (const auto& kv : distances) {
            dist_map[kv.first] = kv.second;
        }
        distance_matrices.push_back(dist_map);
        // Get the distance from the agent's start to goal
        auto it = distances.find(starts[i]);
        int dist = (it != distances.end()) ? it->second : 0;
        goal_distances.push_back(dist);
    }
    // The makespan is the maximum distance required by any agent
    int max_timesteps = 0;
    for (int d : goal_distances) {
        if (d > max_timesteps) max_timesteps = d;
    }
    return {distance_matrices, max_timesteps};
}

/**
 * Creates MDDs for each agent using the map, starts, goals, max_timesteps, and distance matrices.
 * Returns a vector of shared_ptr<MDD>, one for each agent.
 */
std::vector<std::shared_ptr<MDD>>
SATSolverManager::create_mdds(const std::vector<std::vector<char>>& map,
                              const std::vector<std::pair<int, int>>& starts,
                              const std::vector<std::pair<int, int>>& goals,
                              int max_timesteps,
                              const std::vector<std::map<std::pair<int, int>, int>>& distance_matrices)
{
    std::vector<std::shared_ptr<MDD>> mdds;
    for (size_t i = 0; i < starts.size(); ++i) {
        // Convert std::map to PositionDistanceMap for MDDConstructor
        PositionDistanceMap dist_umap;
        for (const auto& kv : distance_matrices[i]) {
            dist_umap[kv.first] = kv.second;
        }
        // Construct the MDD for this agent
        MDDConstructor constructor(map, starts[i], goals[i], max_timesteps, dist_umap);
        std::shared_ptr<MDD> mdd = constructor.construct_mdd();
        mdds.push_back(mdd);
    }
    return mdds;
}

/**
 * Generates a unique filename by appending a number if the file already exists.
 * @param base_filename The base filename to use.
 * @return A unique filename that does not exist yet.
 */
std::string SATSolverManager::get_unique_filename(const std::string& base_filename) {
    namespace fs = std::filesystem;
    if (!fs::exists(base_filename)) {
        return base_filename;
    }
    std::string stem, ext;
    size_t dot = base_filename.find_last_of('.');
    if (dot != std::string::npos) {
        stem = base_filename.substr(0, dot);
        ext = base_filename.substr(dot);
    } else {
        stem = base_filename;
        ext = "";
    }
    int counter = 1;
    std::string candidate;
    do {
        candidate = stem + "_" + std::to_string(counter) + ext;
        ++counter;
    } while (fs::exists(candidate));
    return candidate;
}

/**
 * Creates a CNF from the MDDs using CNFConstructor. Optionally saves the CNF to a file.
 * @param mdds Vector of shared_ptr<MDD> for each agent.
 * @param save_to_file If true, saves the CNF to a file.
 * @param filename If saving, the filename to use (if empty, auto-generate).
 * @return Pair of (shared_ptr<CNF>, filename). Filename is empty if not saved.
 */
std::pair<std::shared_ptr<CNF>, std::string>
SATSolverManager::create_and_save_cnf(const std::vector<std::shared_ptr<MDD>>& mdds,
                                      bool save_to_file,
                                      const std::string& filename,
                                      bool lazy_encoding)
{
    // Build a map from agent_id to MDD for CNFConstructor
    AgentMDDMap mdd_map;
    for (size_t i = 0; i < mdds.size(); ++i) {
        mdd_map[static_cast<int>(i)] = mdds[i];
    }
    // Construct the CNF using CNFConstructor
    CNFConstructor cnf_constructor(mdd_map, lazy_encoding);
    CNF cnf = cnf_constructor.construct_cnf();
    std::shared_ptr<CNF> cnf_ptr = std::make_shared<CNF>(cnf);

    std::string out_filename;
    if (save_to_file) {
        // Determine filename
        if (!filename.empty()) {
            out_filename = get_unique_filename(filename);
        } else {
            // Build a more informative default filename
            // Example: cnf_agents3_20240611_153045.cnf
            std::ostringstream oss;
            // Add agent count
            oss << "cnf_agents" << mdds.size();
            // Add timestamp
            auto now = std::chrono::system_clock::now();
            std::time_t now_c = std::chrono::system_clock::to_time_t(now);
            std::tm tm = *std::localtime(&now_c);
            oss << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
            oss << ".cnf";
            out_filename = get_unique_filename(oss.str());
        }
        // Write CNF to file in DIMACS format
        std::ofstream outfile(out_filename);
        if (!outfile) {
            std::cerr << "Error: Could not open file for writing CNF: " << out_filename << std::endl;
        } else {
            outfile << cnf_ptr->to_dimacs();
            outfile.close();
        }
    }
    // Return the CNF and the filename (empty if not saved)
    return {cnf_ptr, out_filename};
}

/**
 * Creates a CNFProbSATConstructor for direct ProbSAT integration.
 * @param mdds Vector of shared_ptr<MDD> for each agent.
 * @param lazy_encoding If true, use lazy encoding (exclude conflict clauses initially).
 * @return Shared pointer to CNFProbSATConstructor.
 */
std::shared_ptr<CNFProbSATConstructor>
SATSolverManager::create_cnf_probsat_constructor(const std::vector<std::shared_ptr<MDD>>& mdds,
                                                bool lazy_encoding) {
    // Build a map from agent_id to MDD for CNFProbSATConstructor
    AgentMDDMap mdd_map;
    for (size_t i = 0; i < mdds.size(); ++i) {
        mdd_map[static_cast<int>(i)] = mdds[i];
    }
    
    // Create and return CNFProbSATConstructor
    auto constructor = std::make_shared<CNFProbSATConstructor>(mdd_map, lazy_encoding);
    constructor->construct_probsat_cnf();
    return constructor;
}

/**
 * Solves a CNF formula using ProbSAT's in-memory API.
 * @param cnf The CNF formula to solve.
 * @param seed Random seed for ProbSAT.
 * @param max_runs Maximum number of runs.
 * @param max_flips Maximum number of flips per run.
 * @param initial_assignment Optional initial assignment (nullptr for random).
 * @return ProbSATSolution containing the results.
 */
ProbSATSolution SATSolverManager::solve_cnf_with_probsat(const CNF& cnf,
                                                        long long seed,
                                                        long long max_runs,
                                                        long long max_flips,
                                                        const std::vector<int>* initial_assignment) {
    ProbSATSolution result;
    result.satisfiable = false;
    result.num_flips = 0;
    result.solve_time = 0.0;
    result.error_message = "";
    
    try {
        // Convert CNF to ProbSAT format
        std::vector<int*> clause_pointers;
        std::vector<std::vector<int>> clause_storage;
        cnf_to_probsat_format(cnf, clause_pointers, clause_storage);
        
        if (clause_pointers.size() <= 1) {
            result.error_message = "No valid clauses found for ProbSAT";
            return result;
        }
        
        // Prepare ProbSAT result struct
        ProbSatResult probsat_result;
        memset(&probsat_result, 0, sizeof(probsat_result));
        
        // Prepare initial assignment if provided
        std::vector<int> initial_assignment_vec;
        int* initial_assignment_ptr = nullptr;
        if (initial_assignment != nullptr) {
            // ProbSAT expects 1-based indexing, so we need to adjust
            initial_assignment_vec.resize(cnf.count_variables() + 1, 0);
            for (size_t i = 0; i < initial_assignment->size(); ++i) {
                if (i + 1 < initial_assignment_vec.size()) {
                    initial_assignment_vec[i + 1] = (*initial_assignment)[i];
                }
            }
            initial_assignment_ptr = initial_assignment_vec.data();
        }
        
        // Solve with ProbSAT
        auto start_time = std::chrono::high_resolution_clock::now();
        
        int sat = probsat_solve_in_memory(
            cnf.count_variables(),
            cnf.count_clauses(),
            const_cast<int**>(clause_pointers.data()),
            seed,
            max_runs,
            max_flips,
            &probsat_result,
            initial_assignment_ptr
        );
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        // Process results
        result.satisfiable = (sat == 10 || probsat_result.sat == 10);
        result.solve_time = probsat_result.solve_time;
        result.num_flips = probsat_result.num_flips;
        
        if (result.satisfiable && probsat_result.assignment != nullptr) {
            // Copy assignment directly (ProbSAT uses 0-based indexing)
            result.assignment.resize(cnf.count_variables());
            
            // Debug: Print raw ProbSAT assignment
            std::cerr << "[DEBUG] Raw ProbSAT assignment (first 10 elements): ";
            for (int i = 0; i < std::min(10, cnf.count_variables()); ++i) {
                std::cerr << probsat_result.assignment[i] << " ";
            }
            std::cerr << std::endl;
            
            // Copy directly without indexing conversion
            for (int i = 0; i < cnf.count_variables(); ++i) {
                result.assignment[i] = probsat_result.assignment[i];
            }
            
            // Debug: Print processed assignment
            std::cerr << "[DEBUG] Processed assignment: ";
            for (int i = 0; i < std::min(10, (int)result.assignment.size()); ++i) {
                std::cerr << result.assignment[i] << " ";
            }
            std::cerr << std::endl;
        }
        
        // Cleanup
        if (probsat_result.assignment) {
            free(probsat_result.assignment);
        }
        
    } catch (const std::exception& e) {
        result.error_message = std::string("Exception during ProbSAT solving: ") + e.what();
    }
    
    return result;
}

/**
 * Solves a CNF formula using MiniSAT.
 */
MiniSatSolution SATSolverManager::solve_cnf_with_minisat(const CNF& cnf,
                                                        const std::vector<int>* initial_assignment) {
    MiniSatSolution result;
    result.satisfiable = false;
    result.num_decisions = 0;
    result.solve_time = 0.0;
    result.error_message = "";
    
    try {
        // Create MiniSAT wrapper
        MockMiniSatWrapper minisat_wrapper;
        
        // Convert CNF to vector format
        std::vector<std::vector<int>> clauses = cnf.get_clauses();
        
        // Solve with MiniSAT
        result = minisat_wrapper.solve_cnf(clauses, initial_assignment);
        
    } catch (const std::exception& e) {
        result.error_message = std::string("Exception during MiniSAT solving: ") + e.what();
    }
    
    return result;
}

/**
 * Solves a CNF formula using MiniSAT via CNFProbSATConstructor.
 */
MiniSatSolution SATSolverManager::solve_cnf_with_minisat(const std::shared_ptr<CNFProbSATConstructor>& cnf_constructor,
                                                        const std::vector<int>* initial_assignment) {
    MiniSatSolution result;
    result.satisfiable = false;
    result.num_decisions = 0;
    result.solve_time = 0.0;
    result.error_message = "";
    
    try {
        // Create MiniSAT wrapper
        MockMiniSatWrapper minisat_wrapper;
        
        // Get CNF from constructor
        const CNF& cnf = cnf_constructor->get_cnf();
        
        // Convert CNF to vector format
        std::vector<std::vector<int>> clauses = cnf.get_clauses();
        
        // Solve with MiniSAT
        result = minisat_wrapper.solve_cnf(clauses, initial_assignment);
        
    } catch (const std::exception& e) {
        result.error_message = std::string("Exception during MiniSAT solving: ") + e.what();
    }
    
    return result;
}

/**
 * Solves a CNF formula using ProbSAT's in-memory API via CNFProbSATConstructor.
 * @param cnf_constructor The CNFProbSATConstructor to use.
 * @param seed Random seed for ProbSAT.
 * @param max_runs Maximum number of runs.
 * @param max_flips Maximum number of flips per run.
 * @param initial_assignment Optional initial assignment (nullptr for random).
 * @return ProbSATSolution containing the results.
 */
ProbSATSolution SATSolverManager::solve_cnf_with_probsat(const std::shared_ptr<CNFProbSATConstructor>& cnf_constructor,
                                                        long long seed,
                                                        long long max_runs,
                                                        long long max_flips,
                                                        const std::vector<int>* initial_assignment) {
    ProbSATSolution result;
    result.satisfiable = false;
    result.num_flips = 0;
    result.solve_time = 0.0;
    result.error_message = "";
    
    try {
        // Get ProbSAT data from CNFProbSATConstructor
        int** clause_pointers = cnf_constructor->get_probsat_clause_pointers();
        int num_vars = cnf_constructor->get_probsat_num_variables();
        int num_clauses = cnf_constructor->get_probsat_num_clauses();
        
        if (clause_pointers == nullptr || num_clauses == 0) {
            result.error_message = "No valid clauses found for ProbSAT";
            return result;
        }
        
        // Prepare ProbSAT result struct
        ProbSatResult probsat_result;
        memset(&probsat_result, 0, sizeof(probsat_result));
        
        // Prepare initial assignment if provided
        std::vector<int> initial_assignment_vec;
        int* initial_assignment_ptr = nullptr;
        if (initial_assignment != nullptr) {
            // CNFProbSATConstructor uses 0-based indexing
            initial_assignment_vec = *initial_assignment;
            initial_assignment_ptr = initial_assignment_vec.data();
        }
        
        // Solve with ProbSAT
        auto start_time = std::chrono::high_resolution_clock::now();
        
        int sat = probsat_solve_in_memory(
            num_vars,
            num_clauses,
            clause_pointers,
            seed,
            max_runs,
            max_flips,
            &probsat_result,
            initial_assignment_ptr
        );
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        // Process results
        result.satisfiable = (sat == 10 || probsat_result.sat == 10);
        result.solve_time = probsat_result.solve_time;
        result.num_flips = probsat_result.num_flips;
        
        if (result.satisfiable && probsat_result.assignment != nullptr) {
            // Copy assignment (CNFProbSATConstructor uses 0-based indexing)
            result.assignment.resize(num_vars);
            for (int i = 0; i < num_vars; ++i) {
                result.assignment[i] = probsat_result.assignment[i];
            }
        }
        
        // Cleanup
        if (probsat_result.assignment) {
            free(probsat_result.assignment);
        }
        
    } catch (const std::exception& e) {
        result.error_message = std::string("Exception during ProbSAT solving: ") + e.what();
    }
    
    return result;
}

/**
 * Converts CNF to ProbSAT format for in-memory solving.
 * @param cnf The CNF formula to convert.
 * @param clause_pointers Output array of pointers to clauses.
 * @param clause_storage Output storage for clause data.
 */
void SATSolverManager::cnf_to_probsat_format(const CNF& cnf, 
                                            std::vector<int*>& clause_pointers, 
                                            std::vector<std::vector<int>>& clause_storage) {
    int num_vars = cnf.count_variables();
    int clause_idx = 1;
    const auto& all_clauses = cnf.get_clauses();
    int total_clauses = all_clauses.size();
    clause_storage.reserve(total_clauses);
    
    // First, build all clause vectors
    for (const auto& clause : all_clauses) {
        bool is_zero_clause = clause.empty() ||
                              (clause.size() == 1 && clause[0] == 0) ||
                              std::all_of(clause.begin(), clause.end(), [](int lit){ return lit == 0; });
        if (is_zero_clause) {
            std::cerr << "[CNF->ProbSAT] Skipping zero/empty clause at index " << clause_idx << std::endl;
            clause_idx++;
            continue;
        }
        bool valid = true;
        for (int lit : clause) {
            if (lit == 0 || std::abs(lit) > num_vars) {
                std::cerr << "[CNF->ProbSAT] Invalid literal " << lit << " in clause " << clause_idx << ", skipping this clause!" << std::endl;
                valid = false;
                break;
            }
        }
        if (!valid) { 
            clause_idx++; 
            continue; 
        }
        std::vector<int> clause_copy = clause;
        clause_copy.push_back(0); // null terminator
        clause_storage.push_back(clause_copy);
        clause_idx++;
    }
    
    // Now, build the pointer array
    clause_pointers.clear();
    clause_pointers.push_back(nullptr); // dummy for 1-based indexing
    for (auto& clause_vec : clause_storage) {
        clause_pointers.push_back(clause_vec.data());
    }
    std::cerr << "[CNF->ProbSAT] Total clause pointers: " << clause_pointers.size()-1 << ", total clauses: " << total_clauses << std::endl;
}

/**
 * Extracts agent paths from a ProbSAT solution using CNFConstructor.
 * @param cnf_constructor The CNFConstructor used to create the CNF.
 * @param assignment The variable assignment from ProbSAT.
 * @return Map from agent_id to path (vector of positions).
 */
AgentPaths 
SATSolverManager::extract_agent_paths_from_solution(CNFConstructor& cnf_constructor,
                                                   const std::vector<int>& assignment) {
    return cnf_constructor.cnf_assignment_to_paths(assignment);
}

/**
 * Validates agent paths against their MDDs.
 * @param cnf_constructor The CNFConstructor used to create the CNF.
 * @param agent_paths Map from agent_id to path.
 * @return True if all paths are valid, false otherwise.
 */
bool SATSolverManager::validate_agent_paths(CNFConstructor& cnf_constructor,
                                          const AgentPaths& agent_paths) {
    for (const auto& [agent_id, path] : agent_paths) {
        if (!cnf_constructor.validate_path(agent_id, path)) {
            return false;
        }
    }
    return true;
}

/**
 * Prints agent paths in a readable format.
 * @param agent_paths Map from agent_id to path.
 */
void SATSolverManager::print_agent_paths(const AgentPaths& agent_paths) {
    std::cout << "\n=== Agent Paths ===" << std::endl;
    for (const auto& [agent_id, path] : agent_paths) {
        std::cout << "Agent " << agent_id << " path:" << std::endl;
        for (size_t t = 0; t < path.size(); ++t) {
            std::cout << "  Time " << t << ": (" << path[t].first << ", " << path[t].second << ")" << std::endl;
        }
        std::cout << std::endl;
    }
}

/**
 * Calculates max flips and tries based on CNF size (heuristic).
 * @param cnf The CNF formula.
 * @param base_max_flips Base number of max flips.
 * @param base_max_tries Base number of max tries.
 * @return Pair of (max_flips, max_tries).
 */
std::pair<long long, long long> SATSolverManager::calculate_max_flips_and_tries(const CNF& cnf,
                                                                               long long base_max_flips,
                                                                               long long base_max_tries) {
    int num_vars = cnf.count_variables();
    int num_clauses = cnf.count_clauses();
    
    long long max_flips = base_max_flips * (num_vars / 100);
    long long max_tries = base_max_tries * (num_clauses / 50);
    
    // Ensure minimum values
    max_flips = std::max(max_flips, 1000LL);
    max_tries = std::max(max_tries, 10LL); // we'll probably always do just 1 try
    
    return {max_flips, max_tries};
}

/**
 * Detects vertex collisions (two agents at same position at same time).
 * @param agent_paths Map from agent_id to path (vector of positions).
 * @return Vector of collision tuples (agent1_id, agent2_id, position, timestep).
 */
std::vector<std::tuple<int, int, std::pair<int, int>, int>> 
SATSolverManager::find_vertex_collisions(const AgentPaths& agent_paths) {
    std::vector<std::tuple<int, int, std::pair<int, int>, int>> collisions;
    
    // Find max timesteps from the longest path
    int max_timesteps = 0;
    for (const auto& [agent_id, path] : agent_paths) {
        max_timesteps = std::max(max_timesteps, (int)path.size());
    }
    
    // For each timestep, check for collisions
    for (int timestep = 0; timestep < max_timesteps; ++timestep) {
        // Map from position to list of agents at that position
        PositionAgentMap position_agents;
        
        // Collect all agents at each position for this timestep
        for (const auto& [agent_id, path] : agent_paths) {
            if (timestep < (int)path.size()) {
                auto position = path[timestep];
                position_agents[position].push_back(agent_id);
            }
        }
        
        // Check for collisions (more than one agent at same position)
        for (const auto& [position, agents] : position_agents) {
            if (agents.size() > 1) {
                // Add collision for each pair of agents
                for (size_t i = 0; i < agents.size(); ++i) {
                    for (size_t j = i + 1; j < agents.size(); ++j) {
                        collisions.emplace_back(agents[i], agents[j], position, timestep);
                    }
                }
            }
        }
    }
    
    return collisions;
}

/**
 * Detects edge collisions (agents swapping positions between consecutive timesteps).
 * @param agent_paths Map from agent_id to path (vector of positions).
 * @return Vector of edge collision tuples (agent1_id, agent2_id, pos1, pos2, timestep).
 */
std::vector<std::tuple<int, int, std::pair<int, int>, std::pair<int, int>, int>> 
SATSolverManager::find_edge_collisions(const AgentPaths& agent_paths) {
    std::vector<std::tuple<int, int, std::pair<int, int>, std::pair<int, int>, int>> edge_collisions;
    
    // Find max timesteps from the longest path
    int max_timesteps = 0;
    for (const auto& [agent_id, path] : agent_paths) {
        max_timesteps = std::max(max_timesteps, (int)path.size());
    }
    
    // For each timestep (except the last), check for edge collisions
    for (int timestep = 0; timestep < max_timesteps - 1; ++timestep) {
        // Map from edge (pos1, pos2) to list of agents traversing that edge
        EdgeAgentMap edge_agents;
        
        // Collect all agents traversing each edge for this timestep
        for (const auto& [agent_id, path] : agent_paths) {
            if (timestep + 1 < (int)path.size()) {
                auto pos1 = path[timestep];
                auto pos2 = path[timestep + 1];
                
                // Only consider edges where agents actually move
                if (pos1 != pos2) {
                    // Normalize edge direction (smaller position first)
                    auto edge = (pos1 < pos2) ? std::make_pair(pos1, pos2) : std::make_pair(pos2, pos1);
                    edge_agents[edge].push_back(agent_id);
                }
            }
        }
        
        // Check for edge collisions (more than one agent traversing same edge)
        for (const auto& [edge, agents] : edge_agents) {
            if (agents.size() > 1) {
                // Add edge collision for each pair of agents
                for (size_t i = 0; i < agents.size(); ++i) {
                    for (size_t j = i + 1; j < agents.size(); ++j) {
                        edge_collisions.emplace_back(agents[i], agents[j], edge.first, edge.second, timestep);
                    }
                }
            }
        }
    }
    
    return edge_collisions;
}

/**
 * Detects all collisions (both vertex and edge collisions).
 * @param agent_paths Map from agent_id to path (vector of positions).
 * @return Pair of (vertex_collisions, edge_collisions).
 */
std::pair<std::vector<std::tuple<int, int, std::pair<int, int>, int>>,
           std::vector<std::tuple<int, int, std::pair<int, int>, std::pair<int, int>, int>>> 
SATSolverManager::find_all_collisions(const AgentPaths& agent_paths) {
    auto vertex_collisions = find_vertex_collisions(agent_paths);
    auto edge_collisions = find_edge_collisions(agent_paths);
    return {vertex_collisions, edge_collisions};
}

/**
 * Prints collision information in a readable format.
 * @param vertex_collisions Vector of vertex collision tuples.
 * @param edge_collisions Vector of edge collision tuples.
 */
void SATSolverManager::print_collisions(const std::vector<std::tuple<int, int, std::pair<int, int>, int>>& vertex_collisions,
                                       const std::vector<std::tuple<int, int, std::pair<int, int>, std::pair<int, int>, int>>& edge_collisions) {
    std::cout << "\n=== Collision Detection Results ===" << std::endl;
    
    if (vertex_collisions.empty() && edge_collisions.empty()) {
        std::cout << "No collisions detected!" << std::endl;
        return;
    }
    
    if (!vertex_collisions.empty()) {
        std::cout << "Vertex Collisions (" << vertex_collisions.size() << "):" << std::endl;
        for (const auto& collision : vertex_collisions) {
            int agent1, agent2, timestep;
            std::pair<int, int> position;
            std::tie(agent1, agent2, position, timestep) = collision;
            std::cout << "  Agents " << agent1 << " and " << agent2 
                      << " at position (" << position.first << ", " << position.second 
                      << ") at timestep " << timestep << std::endl;
        }
    }
    
    if (!edge_collisions.empty()) {
        std::cout << "Edge Collisions (" << edge_collisions.size() << "):" << std::endl;
        for (const auto& collision : edge_collisions) {
            int agent1, agent2, timestep;
            std::pair<int, int> pos1, pos2;
            std::tie(agent1, agent2, pos1, pos2, timestep) = collision;
            std::cout << "  Agents " << agent1 << " and " << agent2 
                      << " swapping positions (" << pos1.first << ", " << pos1.second 
                      << ") <-> (" << pos2.first << ", " << pos2.second 
                      << ") at timestep " << timestep << std::endl;
        }
    }
}

/**
 * Generates clauses to prevent edge collisions and adds them to the CNF.
 * @param cnf_constructor The CNFProbSATConstructor to add clauses to.
 * @param edge_collisions Vector of edge collision tuples.
 * @return Number of clauses added.
 */
int SATSolverManager::add_edge_collision_prevention_clauses(std::shared_ptr<CNFProbSATConstructor>& cnf_constructor,
                                                           const std::vector<std::tuple<int, int, std::pair<int, int>, std::pair<int, int>, int>>& edge_collisions) {
    int clauses_added = 0;
    
    for (const auto& collision : edge_collisions) {
        int agent1, agent2, timestep;
        std::pair<int, int> pos1, pos2;
        std::tie(agent1, agent2, pos1, pos2, timestep) = collision;
        
        // Get the variable IDs for the conflicting assignments
        // For edge collision: agents are moving in opposite directions on the same edge
        // The collision detection normalizes edge direction, so we need to find the actual
        // positions that agents are at during the collision timestep
        
        // First, let's find what positions the agents are actually at during this timestep
        // by looking at the variable map to see what variables exist
        const auto& var_map = cnf_constructor->get_variable_map();
        
        std::cout << "[DEBUG] Variable map contains:" << std::endl;
        for (const auto& [key, var_id] : var_map) {
            int agent_id = std::get<0>(key);
            auto pos = std::get<1>(key);
            int t = std::get<2>(key);
            std::cout << "  Var " << var_id << ": agent " << agent_id << ", pos (" << pos.first << "," << pos.second << "), t=" << t << std::endl;
        }
        
        // Find all variables for the collision timestep and timestep+1
        std::vector<std::tuple<int, int, std::pair<int, int>, int>> timestep_vars; // agent_id, var_id, pos, timestep
        for (const auto& [key, var_id] : var_map) {
            int agent_id = std::get<0>(key);
            auto pos = std::get<1>(key);
            int t = std::get<2>(key);
            if (t == timestep || t == timestep + 1) {
                timestep_vars.emplace_back(agent_id, var_id, pos, t);
            }
        }
        
        std::cout << "[DEBUG] Variables at timesteps " << timestep << " and " << (timestep + 1) << ":" << std::endl;
        for (const auto& [agent_id, var_id, pos, t] : timestep_vars) {
            std::cout << "  Agent " << agent_id << " at (" << pos.first << "," << pos.second << ") at t=" << t << " (var " << var_id << ")" << std::endl;
        }
        
        // The collision detection reports the movement (from -> to), but we need
        // to find the actual positions that agents are at during the collision
        // For edge collision, agents are moving in opposite directions on the same edge
        
        // Find the actual positions by looking at the variable map
        // We need to find variables for both agents at both timesteps
        std::vector<int> collision_vars;
        
        // Look for variables for agent1 and agent2 at timestep and timestep+1
        for (const auto& [key, var_id] : var_map) {
            int agent_id = std::get<0>(key);
            auto pos = std::get<1>(key);
            int t = std::get<2>(key);
            
            if ((agent_id == agent1 || agent_id == agent2) && (t == timestep || t == timestep + 1)) {
                collision_vars.push_back(var_id);
                std::cout << "[DEBUG] Found collision variable: " << var_id << " for agent " << agent_id 
                          << " at (" << pos.first << "," << pos.second << ") at t=" << t << std::endl;
            }
        }
        
        if (collision_vars.size() >= 4) {
            std::cout << "[DEBUG] Found " << collision_vars.size() << " collision variables" << std::endl;
            
            // Create a clause that prevents this edge collision
            // The clause says: NOT (all collision variables are true)
            // This is equivalent to: (NOT var1) OR (NOT var2) OR (NOT var3) OR (NOT var4) OR ...
            std::vector<int> clause;
            for (int var : collision_vars) {
                clause.push_back(-var);
            }
            
            // Add the clause to the CNF
            cnf_constructor->add_clause(clause);
            clauses_added++;
            
            std::cout << "Added edge collision prevention clause: ";
            for (int lit : clause) std::cout << lit << " ";
            std::cout << "0" << std::endl;
        } else {
            std::cout << "[DEBUG] Could not find enough collision variables. Found: " << collision_vars.size() << std::endl;
        }
        

    }
    
    return clauses_added;
}

/**
 * Generates clauses to prevent vertex collisions and adds them to the CNF.
 * @param cnf_constructor The CNFProbSATConstructor to add clauses to.
 * @param vertex_collisions Vector of vertex collision tuples.
 * @return Number of clauses added.
 */
int SATSolverManager::add_vertex_collision_prevention_clauses(std::shared_ptr<CNFProbSATConstructor>& cnf_constructor,
                                                             const std::vector<std::tuple<int, int, std::pair<int, int>, int>>& vertex_collisions) {
    int clauses_added = 0;
    
    for (const auto& collision : vertex_collisions) {
        int agent1, agent2, timestep;
        std::pair<int, int> position;
        std::tie(agent1, agent2, position, timestep) = collision;
        
        // Get the variable IDs for the conflicting assignments
        // Agent1 at position at timestep
        auto var1_key = std::make_tuple(agent1, position, timestep);
        // Agent2 at position at timestep
        auto var2_key = std::make_tuple(agent2, position, timestep);
        
        // Get variable IDs from the constructor's variable map
        const auto& var_map = cnf_constructor->get_variable_map();
        
        auto it1 = var_map.find(var1_key);
        auto it2 = var_map.find(var2_key);
        
        if (it1 != var_map.end() && it2 != var_map.end()) {
            
            int var1 = it1->second;
            int var2 = it2->second;
            
            // Create a clause that prevents this specific vertex collision
            // The clause says: NOT (agent1 at position at t AND agent2 at position at t)
            // This is equivalent to: (NOT var1) OR (NOT var2)
            std::vector<int> clause = {-var1, -var2};
            
            // Add the clause to the CNF
            cnf_constructor->add_clause(clause);
            clauses_added++;
            
            std::cout << "Added vertex collision prevention clause: ";
            for (int lit : clause) std::cout << lit << " ";
            std::cout << "0" << std::endl;
        }
    }
    
    return clauses_added;
}

/**
 * Creates an initial assignment by sampling paths, adding collisions, and returning assignment (full or partial)
 * If full_assignment is true, returns a full assignment for all agents (for SLS/ProbSAT). If false, returns a partial assignment (for CDCL solvers).
 * @param mdds Map from agent_id to MDD.
 * @param cnf_constructor The CNFProbSATConstructor to add clauses to.
 * @param full_assignment If true, return a full assignment. If false, return a partial assignment.
 * @param rng_ptr Pointer to a random number generator (optional).
 * @return Vector of variable assignments (0 for false, 1 for true).
 */
std::vector<int> SATSolverManager::create_initial_assignment_with_collisions(
    const std::unordered_map<int, std::shared_ptr<MDD>>& mdds,
    CNFConstructor& cnf_constructor,
    bool full_assignment,
    std::mt19937* rng_ptr) {
    
    // Use provided RNG or create a new one
    std::mt19937 local_rng;
    std::mt19937& rng = rng_ptr ? *rng_ptr : local_rng;
    if (!rng_ptr) {
        local_rng.seed(std::random_device{}());
    }
    
    // Step 1: Sample random paths for each agent from their MDDs
    std::unordered_map<int, std::vector<MDDNode::Position>> sampled_paths;
    for (const auto& agent_mdd_pair : mdds) {
        int agent_id = agent_mdd_pair.first;
        const auto& mdd = agent_mdd_pair.second;
        sampled_paths[agent_id] = mdd->sample_random_path(rng);
    }
    
    // Step 2: Detect vertex and edge collisions
    std::vector<std::pair<int, int>> colliding_agents;
    
    // Check vertex collisions (agents at same position and time)
    for (int agent1 = 0; agent1 < mdds.size(); ++agent1) {
        for (int agent2 = agent1 + 1; agent2 < mdds.size(); ++agent2) {
            const auto& path1 = sampled_paths[agent1];
            const auto& path2 = sampled_paths[agent2];
            
            // Check for vertex collisions
            for (size_t t = 0; t < std::min(path1.size(), path2.size()); ++t) {
                if (path1[t] == path2[t]) {
                    colliding_agents.emplace_back(agent1, agent2);
                    // Add vertex collision clause
                    try {
                        cnf_constructor.add_single_collision_clause(agent1, agent2, path1[t], t, true);
                    } catch (const std::exception& e) {
                        // Skip if the clause cannot be added (e.g., position not in MDD)
                        std::cerr << "Warning: Could not add vertex collision clause: " << e.what() << std::endl;
                    }
                }
            }
            
            // Check for edge collisions (agents swap positions)
            for (size_t t = 0; t < std::min(path1.size() - 1, path2.size() - 1); ++t) {
                if (path1[t] == path2[t + 1] && path1[t + 1] == path2[t]) {
                    colliding_agents.emplace_back(agent1, agent2);
                    // Add edge collision clause
                    try {
                        cnf_constructor.add_single_edge_collision_clause(agent1, agent2, path1[t], path1[t + 1], t, true);
                    } catch (const std::exception& e) {
                        // Skip if the clause cannot be added (e.g., edge not in MDD)
                        std::cerr << "Warning: Could not add edge collision clause: " << e.what() << std::endl;
                    }
                }
            }
        }
    }
    
    // Step 3: Ensure CNF is constructed and variables are available
    // This is crucial for lazy encoding - we need to construct the CNF to get variables
    cnf_constructor.construct_cnf();
    
    // Step 4: Create assignment based on mode
    if (full_assignment) {
        // For SLS/ProbSAT: return full assignment for all agents
        std::vector<int> full_assignment_vec;
        for (const auto& agent_path_pair : sampled_paths) {
            int agent_id = agent_path_pair.first;
            const auto& path = agent_path_pair.second;
            
            // Add variables for this agent's path
            for (size_t t = 0; t < path.size(); ++t) {
                int var_id = cnf_constructor.get_variable_id(agent_id, path[t], t);
                full_assignment_vec.push_back(var_id);
            }
        }
        return full_assignment_vec;
    } else {
        // For CDCL solvers: return partial assignment (only non-colliding agents)
        std::unordered_map<int, std::vector<MDDNode::Position>> partial_paths;
        
        // Create a set of agents involved in collisions
        std::set<int> colliding_agent_set;
        for (const auto& collision_pair : colliding_agents) {
            colliding_agent_set.insert(collision_pair.first);
            colliding_agent_set.insert(collision_pair.second);
        }
        
        // Only include agents that are NOT involved in collisions
        for (const auto& agent_path_pair : sampled_paths) {
            int agent_id = agent_path_pair.first;
            if (colliding_agent_set.find(agent_id) == colliding_agent_set.end()) {
                partial_paths[agent_id] = agent_path_pair.second;
            } else {
                partial_paths[agent_id] = {}; // Empty path for colliding agents
            }
        }
        
        return cnf_constructor.partial_assignment_from_paths(partial_paths);
    }
    }

void SATSolverManager::log_run_summary(
    const std::string& log_filename,
    const std::string& map_name,
    int num_agents,
    const std::string& solver_used,
    int cnf_vars_start,
    int cnf_clauses_start,
    int cnf_vars_end,
    int cnf_clauses_end,
    double total_time_s,
    double cnf_build_time_s,
    double total_solver_time_s,
    const std::vector<double>& solver_times_per_iter,
    const std::vector<int>& flips_per_iter,
    const std::vector<int>& tries_per_iter,
    const std::vector<int>& collisions_per_iter,
    const std::string& status,
    long long seed,
    const std::string& params
) {
    // Helper to check if file exists
    auto file_exists = [](const std::string& name) -> bool {
        struct stat buffer;
        return (stat(name.c_str(), &buffer) == 0);
    };

    bool write_header = !file_exists(log_filename);
    std::ofstream log_file(log_filename, std::ios::app);
    if (!log_file.is_open()) {
        std::cerr << "[LOG] Could not open log file: " << log_filename << std::endl;
        return;
    }
    if (write_header) {
        log_file << "map_name,num_agents,solver,cnf_vars_start,cnf_clauses_start,cnf_vars_end,cnf_clauses_end,total_time_s,cnf_build_time_s,total_solver_time_s,solver_times_per_iter,flips_per_iter,tries_per_iter,collisions_per_iter,status,seed,params\n";
    }
    // Helper to join vector as semicolon-separated string
    auto join_vec = [](const auto& vec) -> std::string {
        std::ostringstream oss;
        for (size_t i = 0; i < vec.size(); ++i) {
            oss << vec[i];
            if (i + 1 < vec.size()) oss << ";";
        }
        return oss.str();
    };
    log_file << '"' << map_name << '"' << ','
             << num_agents << ','
             << '"' << solver_used << '"' << ','
             << cnf_vars_start << ','
             << cnf_clauses_start << ','
             << cnf_vars_end << ','
             << cnf_clauses_end << ','
             << total_time_s << ','
             << cnf_build_time_s << ','
             << total_solver_time_s << ','
             << '"' << join_vec(solver_times_per_iter) << '"' << ','
             << '"' << join_vec(flips_per_iter) << '"' << ','
             << '"' << join_vec(tries_per_iter) << '"' << ','
             << '"' << join_vec(collisions_per_iter) << '"' << ','
             << '"' << status << '"' << ','
             << seed << ','
             << '"' << params << '"' << '\n';
    log_file.close();
}

void SATSolverManager::log_timestep_iteration(
    const std::string& log_filename,
    const std::string& map_name,
    int num_agents,
    const std::string& solver_used,
    int timestep,
    int cnf_vars,
    int cnf_clauses,
    double cnf_build_time_s,
    double total_solver_time_s,
    int num_collision_iterations,
    const std::string& status,
    long long seed,
    const std::string& params
) {
    auto file_exists = [](const std::string& name) -> bool {
        struct stat buffer;
        return (stat(name.c_str(), &buffer) == 0);
    };
    bool write_header = !file_exists(log_filename);
    std::ofstream log_file(log_filename, std::ios::app);
    if (!log_file.is_open()) {
        std::cerr << "[LOG] Could not open timestep log file: " << log_filename << std::endl;
        return;
    }
    if (write_header) {
        log_file << "map_name,num_agents,solver,timestep,cnf_vars,cnf_clauses,cnf_build_time_s,total_solver_time_s,num_collision_iterations,status,seed,params\n";
    }
    log_file << '"' << map_name << '"' << ','
             << num_agents << ','
             << '"' << solver_used << '"' << ','
             << timestep << ','
             << cnf_vars << ','
             << cnf_clauses << ','
             << cnf_build_time_s << ','
             << total_solver_time_s << ','
             << num_collision_iterations << ','
             << '"' << status << '"' << ','
             << seed << ','
             << '"' << params << '"' << '\n';
    log_file.close();
}

void SATSolverManager::log_collision_iteration(
    const std::string& log_filename,
    const std::string& map_name,
    int num_agents,
    const std::string& solver_used,
    int timestep,
    int collision_iter,
    int cnf_vars,
    int cnf_clauses,
    double solver_time_s,
    int flips,
    int tries,
    int collisions_added,
    const std::string& status,
    long long seed,
    const std::string& params
) {
    auto file_exists = [](const std::string& name) -> bool {
        struct stat buffer;
        return (stat(name.c_str(), &buffer) == 0);
    };
    bool write_header = !file_exists(log_filename);
    std::ofstream log_file(log_filename, std::ios::app);
    if (!log_file.is_open()) {
        std::cerr << "[LOG] Could not open collision log file: " << log_filename << std::endl;
        return;
    }
    if (write_header) {
        log_file << "map_name,num_agents,solver,timestep,collision_iter,cnf_vars,cnf_clauses,solver_time_s,flips,tries,collisions_added,status,seed,params\n";
    }
    log_file << '"' << map_name << '"' << ','
             << num_agents << ','
             << '"' << solver_used << '"' << ','
             << timestep << ','
             << collision_iter << ','
             << cnf_vars << ','
             << cnf_clauses << ','
             << solver_time_s << ','
             << flips << ','
             << tries << ','
             << collisions_added << ','
             << '"' << status << '"' << ','
             << seed << ','
             << '"' << params << '"' << '\n';
    log_file.close();
}

 