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
        // Convert std::map to std::unordered_map for MDDConstructor
        std::unordered_map<MDDNode::Position, int, pair_hash> dist_umap;
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
                                      const std::string& filename)
{
    // Build a map from agent_id to MDD for CNFConstructor
    std::unordered_map<int, std::shared_ptr<MDD>> mdd_map;
    for (size_t i = 0; i < mdds.size(); ++i) {
        mdd_map[static_cast<int>(i)] = mdds[i];
    }
    // Construct the CNF using CNFConstructor
    CNFConstructor cnf_constructor(mdd_map);
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