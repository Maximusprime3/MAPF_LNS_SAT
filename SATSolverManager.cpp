#include <cmath>
#include <limits>
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


std::vector<std::vector<char>> SATSolverManager::load_map(const std::string& map_path) {
    std::ifstream infile(map_path);
    std::vector<std::vector<char>> map;
    if (!infile) {
        std::cerr << "Error: Could not open map file: " << map_path << std::endl;
        return map;
    }

    std::string line;
    auto read_line = [&]() {
        if (!std::getline(infile, line)) return false;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        return true;
    };
    auto read_dimension = [&](const char* expected, int& value) {
        if (!read_line()) return false;
        std::istringstream header(line);
        std::string key, extra;
        return (header >> key >> value) && key == expected && value > 0 && !(header >> extra);
    };
    int height = 0, width = 0;
    if (!read_line() || line != "type octile" ||
        !read_dimension("height", height) || !read_dimension("width", width) ||
        !read_line() || line != "map") {
        std::cerr << "Error: Invalid Moving AI map header: " << map_path << std::endl;
        return {};
    }
    bool has_swamp_or_water = false;
    while (read_line()) {
        if (line.size() != static_cast<std::size_t>(width) || map.size() >= static_cast<std::size_t>(height)) {
            std::cerr << "Error: Map rows do not match declared dimensions: " << map_path << std::endl;
            return {};
        }
        for (char c : line) {
            if (c == 'S' || c == 'W') has_swamp_or_water = true;
        }
        map.emplace_back(line.begin(), line.end());
    }
    if (infile.bad() || map.size() != static_cast<std::size_t>(height)) {
        std::cerr << "Error: Incomplete map data: " << map_path << std::endl;
        return {};
    }
    if (has_swamp_or_water) {
        std::cerr << "Warning: Swamp (S) and Water (W) will be handled as unpassable terrain" << std::endl;
    }
    return map;
}

//just the window
std::vector<std::vector<char>> SATSolverManager::crop_map_window(
    const std::vector<std::vector<char>>& map,
    const std::pair<int,int>& center,
    int offset) {
    std::vector<std::vector<char>> window;
    if (map.empty() || map[0].empty() || offset < 0) return window;

    const int rows = static_cast<int>(map.size());
    const int cols = static_cast<int>(map[0].size());

    // Clamp center to map bounds in case the provided center is out of range
    const int c_r = std::max(0, std::min(center.first, rows - 1));
    const int c_c = std::max(0, std::min(center.second, cols - 1));

    // Compute clamped window bounds (inclusive)
    const int r0 = std::max(0, c_r - offset);
    const int c0 = std::max(0, c_c - offset);
    const int r1 = std::min(rows - 1, c_r + offset);
    const int c1 = std::min(cols - 1, c_c + offset);

    // Reserve and copy
    window.reserve(static_cast<size_t>(r1 - r0 + 1));
    for (int r = r0; r <= r1; ++r) {
        std::vector<char> row;
        row.reserve(static_cast<size_t>(c1 - c0 + 1));
        for (int c = c0; c <= c1; ++c) {
            row.push_back(map[r][c]);
        }
        window.push_back(std::move(row));
    }

    return window;
}
//the whole map but everything but the window is unwalkable (constant coordinates for the window)
std::vector<std::vector<char>> SATSolverManager::mask_map_outside_window(
    const std::vector<std::vector<char>>& map,
    const std::pair<int,int>& center,
    int offset) {
    if (map.empty() || map[0].empty() || offset < 0) return {};

    const int rows = static_cast<int>(map.size());
    const int cols = static_cast<int>(map[0].size());

    // Clamp center to map bounds
    const int c_r = std::max(0, std::min(center.first, rows - 1));
    const int c_c = std::max(0, std::min(center.second, cols - 1));

    // Compute clamped window bounds (inclusive)
    const int r0 = std::max(0, c_r - offset);
    const int c0 = std::max(0, c_c - offset);
    const int r1 = std::min(rows - 1, c_r + offset);
    const int c1 = std::min(cols - 1, c_c + offset);

    // Create full-size map filled with unwalkable '@' and then override window
    std::vector<std::vector<char>> masked(rows, std::vector<char>(cols, '@'));

    // Copy only the window region from the original map
    for (int r = r0; r <= r1; ++r) {
        for (int c = c0; c <= c1; ++c) {
            masked[r][c] = map[r][c];
        }
    }

    return masked;
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
        if (!line.empty() && line.back() == '\r') line.pop_back();
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
            return {}; // Never shift scenario slices by silently dropping a row.
        }
        ScenarioEntry entry;
        try {
            auto strict_int = [](const std::string& text) {
                std::size_t used = 0;
                const int value = std::stoi(text, &used);
                if (used != text.size()) throw std::invalid_argument("Trailing text in scenario integer");
                return value;
            };
            // Convert numeric fields from string to int, assign map_name as string
            entry.bucket = strict_int(tokens[0]);
            entry.map_name = tokens[1];
            entry.map_width = strict_int(tokens[2]);
            entry.map_height = strict_int(tokens[3]);
            entry.start_x = strict_int(tokens[5]);
            entry.start_y = strict_int(tokens[4]); //movingai is (y, x)
            entry.goal_x = strict_int(tokens[7]);
            entry.goal_y = strict_int(tokens[6]);
            std::size_t used = 0;
            const double optimal_length = std::stod(tokens[8], &used);
            if (used != tokens[8].size() || !std::isfinite(optimal_length) ||
                optimal_length < 0 || optimal_length > std::numeric_limits<int>::max())
                throw std::invalid_argument("Invalid optimal length in scenario");
            entry.optimal_length = static_cast<int>(optimal_length);
            //print entry
            //std::cout << "Entry: " << entry.start_x << "," << entry.start_y << " -> " << entry.goal_x << "," << entry.goal_y << std::endl;
            //map width and height
            //std::cout << "Map width: " << entry.map_width << ", Map height: " << entry.map_height << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Warning: Failed to parse scenario line: " << line << "\n" << e.what() << std::endl;
            return {};
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
    const std::vector<std::pair<int, int>>& goals, SolverDeadline deadline)
{
    std::vector<std::map<std::pair<int, int>, int>> distance_matrices;
    std::vector<int> goal_distances;

    // For each agent, compute the distance matrix and the distance from start to goal
    for (size_t i = 0; i < starts.size(); ++i) {
        MDDConstructor constructor(map, starts[i], goals[i], -1, {}, deadline);
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


CNFConstructor
SATSolverManager::create_cnf_constructor(const std::vector<std::shared_ptr<MDD>>& mdds,
                                        bool lazy_encoding) {
    // Build a map from agent_id to MDD for CNFConstructor
    AgentMDDMap mdd_map;
    for (size_t i = 0; i < mdds.size(); ++i) {
        mdd_map[static_cast<int>(i)] = mdds[i];
    }
    
    // Create and return CNFConstructor
    CNFConstructor constructor(mdd_map, lazy_encoding);
    return constructor;
}




/**
 * Extracts agent paths from a SAT assignment using CNFConstructor.
 * @param cnf_constructor The CNFConstructor used to create the CNF.
 * @param assignment The SAT variable assignment.
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
                //find if any other agent is at the same position
                auto it = position_agents.find(position);
                if (it != position_agents.end()) {
                    //add collision for each other agent at the same position
                    for (int other_agent : it->second) {
                        collisions.emplace_back(other_agent, agent_id, position, timestep);
                    }
                }
                //store agent position so if any other agent gets there at the same time we can find the vertex collision
                position_agents[position].push_back(agent_id);
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

    // For each timestep (except the last), detect true swaps (opposite edges)
    for (int timestep = 0; timestep < max_timesteps - 1; ++timestep) {
        // Collect all actual moves (from != to) at this timestep
        EdgeAgentMap edge_map;
        edge_map.reserve(agent_paths.size());

        for (const auto& [agent_id, path] : agent_paths) {
            if (timestep + 1 < (int)path.size()) {
                auto from = path[timestep];
                auto to = path[timestep + 1];
                if (from != to) {
                    auto edge = std::make_pair(from, to);
                    auto rev_edge = std::make_pair(to, from);

                    auto it = edge_map.find(rev_edge);
                    if (it != edge_map.end()) {
                        for (int other_agent : it->second) {
                            //we will find all edge collisions when we check the second agent that is part of the edge collision
                            edge_collisions.emplace_back(other_agent, agent_id,
                                                        rev_edge.first, rev_edge.second,
                                                        timestep);
                        }
                    }
                    //store movement of agent so if any other agent moves like that in reverse we can find the edge collision
                    edge_map[edge].push_back(agent_id);
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

void SATSolverManager::log_collision_iteration_minisat(
    const std::string& log_filename,
    const std::string& map_name,
    int num_agents,
    const std::string& solver_used,
    int timestep,
    int collision_iter,
    int cnf_vars,
    int cnf_clauses,
    double solver_time_s,
    int decisions,
    int propagations,
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
        log_file << "map_name,num_agents,solver,timestep,collision_iter,cnf_vars,cnf_clauses,solver_time_s,decisions,propagations,collisions_added,status,seed,params\n";
    }
    log_file << '"' << map_name << '"' << ','
             << num_agents << ','
             << '"' << solver_used << '"' << ','
             << timestep << ','
             << collision_iter << ','
             << cnf_vars << ','
             << cnf_clauses << ','
             << solver_time_s << ','
             << decisions << ','
             << propagations << ','
             << collisions_added << ','
             << '"' << status << '"' << ','
             << seed << ','
             << '"' << params << '"' << '\n';
    log_file.close();
}

void SATSolverManager::log_run_summary_minisat(
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
    const std::vector<int>& decisions_per_iter,
    const std::vector<int>& propagations_per_iter,
    const std::vector<int>& collisions_per_iter,
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
        std::cerr << "[LOG] Could not open run summary log file: " << log_filename << std::endl;
        return;
    }
    if (write_header) {
        log_file << "map_name,num_agents,solver,cnf_vars_start,cnf_clauses_start,cnf_vars_end,cnf_clauses_end,total_time_s,cnf_build_time_s,total_solver_time_s,solver_times_per_iter,decisions_per_iter,propagations_per_iter,collisions_per_iter,status,seed,params\n";
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
             << '"' << join_vec(decisions_per_iter) << '"' << ','
             << '"' << join_vec(propagations_per_iter) << '"' << ','
             << '"' << join_vec(collisions_per_iter) << '"' << ','
             << '"' << status << '"' << ','
             << seed << ','
             << '"' << params << '"' << '\n';
    log_file.close();
}


void SATSolverManager::log_timestep_iteration_minisat(
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
    int decisions,
    int propagations,
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
        log_file << "map_name,num_agents,solver,timestep,cnf_vars,cnf_clauses,cnf_build_time_s,total_solver_time_s,num_collision_iterations,decisions,propagations,status,seed,params\n";
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
             << decisions << ','
             << propagations << ','
             << '"' << status << '"' << ','
             << seed << ','
             << '"' << params << '"' << '\n';
    log_file.close();
}

