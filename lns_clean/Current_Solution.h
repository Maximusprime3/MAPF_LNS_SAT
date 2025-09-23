#pragma once

#include <unordered_map>
#include <vector>
#include <set>
#include <optional>
#include <algorithm>
#include <iostream>

// Shared LNS core structures used across solver and helpers

// Metadata describing a single conflict between two agents.
// This is useful when building conflict buckets or applying waiting time
// strategies where we only need to reason about the agents involved and the
// timesteps of the conflicts.
struct ConflictMeta {
    int agent1;
    int agent2;
    int timestep;
    bool is_edge;                 // true for edge conflict, false for vertex
    std::pair<int,int> pos1;      // vertex position or first edge endpoint
    std::pair<int,int> pos2;      // second edge endpoint if edge conflict
};

// Represents a diamond shaped conflict bucket.  Each bucket stores the set of
// grid positions that belong to the diamond as well as the indices of the
// conflicts contained within the bucket.  Buckets are used to isolate regions
// of the map for local solving.
struct DiamondBucket {
    std::set<std::pair<int,int>> positions;
    std::vector<int> indices;
    std::vector<std::vector<char>> masked_map;
    int earliest_t; // earliest timestep among conflicts in this bucket
    int latest_t; // latest timestep among conflicts in this bucket
};



struct CurrentSolution {
    std::unordered_map<int, std::vector<std::pair<int,int>>> agent_paths;  // agent_id -> path
    // Map flattened (row, col, timestep) -> list of agents occupying that cell at that time
    // Key is encoded as ((row * cols) + col) * (max_timestep + 1) + t
    std::unordered_map<long long, std::vector<int>> path_map;
    int max_timestep;
    int rows, cols;  // Store dimensions for bounds checking
    // Waiting time tracking: agent_id -> number of timesteps spent waiting at goal
    std::unordered_map<int, int> agent_waiting_time;
    
    CurrentSolution(int rows_, int cols_, int max_t, int num_agents = 50) 
        : max_timestep(std::max(0, max_t)), rows(rows_), cols(cols_) {
        // Reserve space for the actual number of agents to avoid rehashing
        // Default to 50 if not specified, which is reasonable for most MAPF scenarios
        agent_paths.reserve(num_agents);
        agent_waiting_time.reserve(num_agents);
    }

    // Helper to encode 3D coordinates into a single key for path_map
    long long encode_key(int r, int c, int t) const {
        return (static_cast<long long>(r) * cols + c) * (max_timestep + 1LL) + t;
    }
    
    // Create path map by "drawing" all agent paths including timesteps
    void create_path_map() {
        // Clear existing path map
        path_map.clear();
        
        for (const auto& [agent_id, path] : agent_paths) {
            for (size_t t = 0; t < path.size(); ++t) {
                auto [r, c] = path[t];
                if (r >= 0 && r < rows && c >= 0 && c < cols) {
                    int tt = static_cast<int>(t);
                    if (tt <= max_timestep) {
                        long long key = encode_key(r, c, tt);
                        path_map[key].push_back(static_cast<int>(agent_id));
                    }
                }
            }
        }
    }
    
    // Find agents that pass through a specific position at a specific time
    std::vector<int> get_agents_at_position_time(int r, int c, int t) const {
        if (r >= 0 && r < rows && c >= 0 && c < cols && t >= 0 && t <= max_timestep) {
            long long key = encode_key(r, c, t);
            auto it = path_map.find(key);
            if (it != path_map.end()) {
                return it->second;
            }
        }
        return {};
    }
    
    // Find agents that pass through any position in the conflict zone at any time
    //AHAHH only need agents that pass through the zone at conflict time +- offset
   // Find agents that pass through any position in the conflict zone
    // Optionally restrict the search to a specific timestep window
    std::set<int> get_agents_in_zone(const std::set<std::pair<int,int>>& zone_positions,
                                    std::optional<int> start_t = std::nullopt,
                                    std::optional<int> end_t   = std::nullopt) const {
            std::set<int> agents_in_zone;

            // Determine the effective time window to scan
            int begin = start_t.value_or(0);
            int finish = end_t.value_or(max_timestep);
            begin = std::max(0, begin);
            finish = std::min(max_timestep, finish);

            for (const auto& [r, c] : zone_positions) {
                if (r < 0 || r >= rows || c < 0 || c >= cols) continue;
                for (int t = begin; t <= finish; ++t) {
                    long long key = encode_key(r, c, t);
                    auto it = path_map.find(key);
                    if (it != path_map.end()) {
                        agents_in_zone.insert(it->second.begin(), it->second.end());        
                    }
                }
            }
        return agents_in_zone;
    }
    
    // Update global solution with local paths from conflict zone resolution
    // Replaces the conflicting segments in agent paths with collision-free local paths
    void update_with_local_paths(const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_paths,
                                const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time) {
        std::cout << "[LNS] Updating global solution with local paths..." << std::endl;
        
        for (const auto& [agent_id, local_path] : local_paths) {
            auto entry_exit = local_entry_exit_time.at(agent_id);
            int entry_t = entry_exit.first;
            int exit_t = entry_exit.second;
            
            // Get the agent's global path
            auto global_it = agent_paths.find(agent_id);
            if (global_it == agent_paths.end()) {
                std::cerr << "[ERROR] Agent " << agent_id
                          << " missing from global solution when applying local path update" << std::endl;
                continue;
            }
            auto& global_path = global_it->second;
            
            // Replace the segment from entry_t to exit_t with the local path
            // The local path should have the same length as the segment it replaces
            int segment_length = exit_t - entry_t + 1;
            
            // Bounds checking: ensure entry_t and exit_t are within global_path bounds
            if (entry_t < 0 || exit_t >= (int)global_path.size() || entry_t > exit_t) {
                std::cerr << "[ERROR] Agent " << agent_id << " invalid time bounds: entry_t=" << entry_t 
                          << ", exit_t=" << exit_t << ", global_path_size=" << global_path.size() << std::endl;
                continue;
            }
            
            if ((int)local_path.size() == segment_length) {
                // Replace the segment in the global path with bounds checking
                for (int i = 0; i < segment_length; ++i) {
                    int target_index = entry_t + i;
                    if (target_index >= 0 && target_index < (int)global_path.size()) {
                        global_path[target_index] = local_path[i];
                    } else {
                        std::cerr << "[ERROR] Agent " << agent_id << " target index " << target_index 
                                  << " out of bounds (global_path_size=" << global_path.size() << ")" << std::endl;
                        break;
                    }
                }
                std::cout << "  Updated agent " << agent_id << " path segment from t=" << entry_t 
                          << " to t=" << exit_t << " (length=" << segment_length << ")" << std::endl;
            } else {
                std::cerr << "[ERROR] Agent " << agent_id << " local path length (" << local_path.size() 
                          << ") doesn't match expected segment length (" << segment_length << ")" << std::endl;
            }
        }
        
        // Update the path map to reflect the new paths
        std::cout << "[LNS] Updating path map with new local paths..." << std::endl;
        create_path_map();
        
        std::cout << "[LNS] Successfully updated global solution with local paths!" << std::endl;
    }

    // Update global solution when waiting time was used (local segment lengthened)
    // For each agent: shift the suffix after the original segment forward by delta
    // and then replace the segment with the provided local path spanning the new bounds.
    void update_with_local_paths_waiting(
        const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_paths,
        const std::unordered_map<int, std::pair<int,int>>& original_entry_exit_time,
        const std::unordered_map<int, std::pair<int,int>>& new_entry_exit_time) {
        std::cout << "[LNS] Updating global solution with local paths (waiting time) ..." << std::endl;

        for (const auto& [agent_id, local_path] : local_paths) {
            auto it_new = new_entry_exit_time.find(agent_id);
            if (it_new == new_entry_exit_time.end()) continue;
            int new_entry_t = it_new->second.first;
            int new_exit_t  = it_new->second.second;

            auto it_old = original_entry_exit_time.find(agent_id);
            int old_entry_t = new_entry_t;
            int old_exit_t  = new_exit_t;
            if (it_old != original_entry_exit_time.end()) {
                old_entry_t = it_old->second.first;
                old_exit_t  = it_old->second.second;
            }

            int delta = new_exit_t - old_exit_t;

            // Bounds and size checks on global path
            auto global_it = agent_paths.find(agent_id);
            if (global_it == agent_paths.end()) {
                std::cerr << "[ERROR] Agent " << agent_id
                          << " missing from global solution when applying waiting-time update" << std::endl;
                continue;
            }
            auto& global_path = global_it->second;
            const int N = (int)global_path.size(); // should be makespan as paths are padded to makespan with last position
            if (new_entry_t < 0 || new_entry_t > new_exit_t) {
                std::cerr << "[ERROR] Agent " << agent_id << " invalid new bounds: [" << new_entry_t
                          << "," << new_exit_t << "], global_path_size=" << N << std::endl;
                continue;
            }

            // If waiting time increased the path length, shift the suffix first
            if (delta > 0) {
                if (new_exit_t >= N) {
                    std::cerr << "[ERROR] Agent " << agent_id << " new_exit_t " << new_exit_t
                              << " out of bounds (N=" << N << ")" << std::endl;
                    continue;
                }
                // Shift suffix [old_exit_t+1 .. N-1] forward by delta within fixed length. 
                // Note: this is safe as we pad paths to makespan and can replace waiting at the goal with the path that now takes longer
                for (int t = N - 1; t >= new_exit_t + 1; --t) {
                    int src = t - delta;
                    if (src >= old_exit_t + 1 && src < N) global_path[t] = global_path[src];
                    
                }
            }

            // Now replace the (extended) segment with the new local path
            int segment_length = new_exit_t - new_entry_t + 1;
            if ((int)local_path.size() != segment_length) {
                std::cerr << "[ERROR] Agent " << agent_id << " local path len (" << local_path.size()
                          << ") != expected (" << segment_length << ")" << std::endl;
                continue;
            }
            for (int i = 0; i < segment_length; ++i) {
                int idx = new_entry_t + i;
                if (idx >= 0 && idx < N) global_path[idx] = local_path[i];
            }
        }

        // Rebuild occupancy map
        create_path_map();
        std::cout << "[LNS] Successfully updated global solution with waiting-time local paths!" << std::endl;
    }
    
    // Calculate waiting time for each agent based on their shortest path vs makespan
    // This should be called after initial path sampling to track available extra actions
    void calculate_waiting_times(const std::vector<std::pair<int,int>>& goals, int makespan) {
        std::cout << "[LNS] Calculating waiting times for agents..." << std::endl;
        
        for (const auto& [agent_id, path] : agent_paths) {
            //if (agent_id < 0 || agent_id >= (int)goals.size()) continue;
            
            // Find when the agent reaches its goal
            auto goal_pos = goals[agent_id];
            int goal_reached_time = -1;
            
            for (int t = 0; t < (int)path.size(); ++t) {
                if (path[t] == goal_pos) {
                    goal_reached_time = t;
                    break;
                }
            }
            
            if (goal_reached_time != -1) {
                // Calculate waiting time: makespan - goal_reached_time
                int waiting_time = makespan - goal_reached_time;
                agent_waiting_time[agent_id] = std::max(0, waiting_time);
                
                std::cout << "  Agent " << agent_id << " reaches goal at t=" << goal_reached_time 
                          << ", waiting time=" << agent_waiting_time[agent_id] << std::endl;
            } else {
                // Agent never reaches goal (shouldn't happen with proper MDDs)
                agent_waiting_time[agent_id] = 0;
                std::cerr << "[WARNING] Agent " << agent_id << " never reaches its goal!" << std::endl;
            }
        }
        
        std::cout << "[LNS] Waiting time calculation complete" << std::endl;
    }

    // Ensure every agent path has length max_timestep+1 by padding
    // trailing steps with the last known position (typically the goal)
    void pad_paths_to_makespan() {
        int target_len = max_timestep + 1;
        for (auto& [agent_id, path] : agent_paths) {
            if (path.empty()) continue;
            auto last_pos = path.back();
            while ((int)path.size() < target_len) {
                path.push_back(last_pos);
            }
        }
    }
    
    // Get waiting time available for an agent
    int get_waiting_time(int agent_id) const {
        auto it = agent_waiting_time.find(agent_id);
        return (it != agent_waiting_time.end()) ? it->second : 0;
    }
    
    // Use some waiting time for an agent (reduce available waiting time)
    void use_waiting_time(int agent_id, int timesteps_used) {
        auto it = agent_waiting_time.find(agent_id);
        if (it != agent_waiting_time.end()) {
            it->second = std::max(0, it->second - timesteps_used);
            std::cout << "  Agent " << agent_id << " used " << timesteps_used 
                      << " waiting timesteps, " << it->second << " remaining" << std::endl;
        }
    }
    
    // Backup current waiting times for potential restoration
    std::unordered_map<int, int> backup_waiting_times() const {
        return agent_waiting_time;
    }
    
    // Restore waiting times from backup
    void restore_waiting_times(const std::unordered_map<int, int>& backup) {
        agent_waiting_time = backup;
        std::cout << "[LNS] Restored waiting times from backup" << std::endl;
    }
    
    // Get waiting times for a set of agents, sorted by waiting time (descending)
    std::vector<std::pair<int, int>> get_agents_waiting_times(
        const std::set<int>& agent_ids) const {
        std::vector<std::pair<int, int>> agents_with_waiting;
        
        for (int agent_id : agent_ids) {
            int waiting_time = get_waiting_time(agent_id);
            if (waiting_time > 0) {
                agents_with_waiting.push_back({agent_id, waiting_time});
            }
        }
        
        // Sort by waiting time (descending - most waiting time first)
        std::sort(agents_with_waiting.begin(), agents_with_waiting.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        
        return agents_with_waiting;
    }
};
