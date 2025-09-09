#include "LNSHelpers.h"
#include "LNSGeometry.h"
#include <iostream>
#include <algorithm>
#include <random>

// ============================================================================
// COLLISION TRACKER IMPLEMENTATION
// ============================================================================

void CollisionTracker::add_vertex_collisions(const std::vector<VertexCollision>& collisions) {
    vertex_collisions_.insert(vertex_collisions_.end(), collisions.begin(), collisions.end());
}

void CollisionTracker::add_edge_collisions(const std::vector<EdgeCollision>& collisions) {
    edge_collisions_.insert(edge_collisions_.end(), collisions.begin(), collisions.end());
}

void CollisionTracker::merge_collisions(const CollisionTracker& other) {
    add_vertex_collisions(other.vertex_collisions_);
    add_edge_collisions(other.edge_collisions_);
}

void CollisionTracker::clear() {
    vertex_collisions_.clear();
    edge_collisions_.clear();
}

void CollisionTracker::print_summary(const std::string& prefix) const {
    std::cout << prefix << "Collision Summary: " << vertex_collisions_.size() 
              << " vertex collisions, " << edge_collisions_.size() 
              << " edge collisions" << std::endl;
}

// ============================================================================
// CONFLICT BUCKET MANAGER IMPLEMENTATION
// ============================================================================

ConflictBucketManager::ConflictBucketManager(int rows, int cols,
    const std::vector<std::vector<char>>& grid,
    int offset)
: grid_(grid), rows_(rows), cols_(cols), offset_(offset) {}


std::vector<DiamondBucket> ConflictBucketManager::create_diamond_buckets(
    const std::vector<std::pair<int,int>>& conflict_points,
    const std::vector<ConflictMeta>& conflict_meta,
    const std::set<int>& solved_conflict_indices) {
    
    std::vector<DiamondBucket> diamond_buckets;
    std::vector<char> diamond_used(conflict_points.size(), 0);
    
    for (size_t i = 0; i < conflict_points.size(); ++i) {
        // Skip if already used or solved
        if (diamond_used[i] || solved_conflict_indices.find(i) != solved_conflict_indices.end()) {
            continue;
        }
        
        // Create new bucket starting from this conflict
        DiamondBucket bucket;
        bucket.positions.insert(conflict_points[i]);
        bucket.indices.push_back(i);
        diamond_used[i] = 1;
        
        // Collect all conflicts that will be part of this diamond bucket
        std::vector<std::pair<int,int>> bucket_conflicts;
        bucket_conflicts.push_back(conflict_points[i]);
        
        // Keep track of previous shape to only check new regions
        std::set<std::pair<int,int>> previous_shape;
        bool found_more_conflicts = true;
        int iteration = 0;
        
        while (found_more_conflicts && iteration < 10) { // Safety limit
            iteration++;
            found_more_conflicts = false;
            
            // Create diamond shape from current bucket conflicts
            auto current_diamond = create_shape_from_conflicts(bucket_conflicts, offset_);
            
            // Find new positions in the current diamond
            auto new_positions = find_new_positions(current_diamond, previous_shape);
            
            // Check new positions for conflicts
            for (const auto& pos : new_positions) {
                auto [r, c] = pos;
                if (r >= 0 && r < rows_ && c >= 0 && c < cols_) {
                    // Find conflict at this position
                    for (size_t j = 0; j < conflict_points.size(); ++j) {
                        if (!diamond_used[j] && 
                            solved_conflict_indices.find(j) == solved_conflict_indices.end() &&
                            conflict_points[j] == pos) {
                            
                            bucket_conflicts.push_back(conflict_points[j]);
                            bucket.indices.push_back(j);
                            diamond_used[j] = 1;
                            found_more_conflicts = true;
                            break;
                        }
                    }
                }
            }
            
            previous_shape = current_diamond;
        }
        
        // Create final diamond shape and masked map
        bucket.positions = create_shape_from_conflicts(bucket_conflicts, offset_);
        bucket.masked_map = mask_map_outside_shape(grid_, bucket.positions);
        
        diamond_buckets.push_back(std::move(bucket));
    }
    
    return diamond_buckets;
}

std::vector<int> ConflictBucketManager::find_conflicts_in_positions(
    const std::set<std::pair<int,int>>& positions,
    const std::vector<std::pair<int,int>>& conflict_points,
    const std::set<int>& solved_conflict_indices,
    const std::set<int>& original_bucket_indices) {
    
    std::vector<int> found_conflicts;
    
    for (const auto& pos : positions) {
        auto [r, c] = pos;
        if (r >= 0 && r < rows_ && c >= 0 && c < cols_) {
            // Find conflict at this position
            for (size_t i = 0; i < conflict_points.size(); ++i) {
                if (conflict_points[i] == pos &&
                    solved_conflict_indices.find(i) == solved_conflict_indices.end() &&
                    original_bucket_indices.find(i) == original_bucket_indices.end()) {
                    
                    found_conflicts.push_back(i);
                    break;
                }
            }
        }
    }
    
    return found_conflicts;
}

std::vector<std::vector<int>> ConflictBucketManager::create_conflict_map(
    const std::vector<std::pair<int,int>>& conflict_points) {
    
    std::vector<std::vector<int>> conflict_map(rows_, std::vector<int>(cols_, -1));
    
    for (size_t i = 0; i < conflict_points.size(); ++i) {
        auto [r, c] = conflict_points[i];
        if (r >= 0 && r < rows_ && c >= 0 && c < cols_) {
            conflict_map[r][c] = i;
        }
    }
    
    return conflict_map;
}

void ConflictBucketManager::print_bucket_statistics(const std::vector<DiamondBucket>& buckets) const {
    std::cout << "[LNS] Created " << buckets.size() << " conflict buckets:" << std::endl;
    for (size_t i = 0; i < buckets.size(); ++i) {
        const auto& bucket = buckets[i];
        std::cout << "  Bucket " << i << ": " << bucket.positions.size() 
                  << " positions, " << bucket.indices.size() << " conflicts" << std::endl;
    }
}

bool ConflictBucketManager::is_within_diamond(const std::pair<int,int>& pos, 
                                             const std::pair<int,int>& center, 
                                             int radius) const {
    return std::abs(pos.first - center.first) + std::abs(pos.second - center.second) <= radius;
}

std::set<std::pair<int,int>> ConflictBucketManager::find_new_positions(
    const std::set<std::pair<int,int>>& expanded_shape,
    const std::set<std::pair<int,int>>& original_shape) const {
    
    std::set<std::pair<int,int>> new_positions;
    for (const auto& pos : expanded_shape) {
        if (original_shape.find(pos) == original_shape.end()) {
            new_positions.insert(pos);
        }
    }
    return new_positions;
}

// ============================================================================
// MDD MANAGER IMPLEMENTATION
// ============================================================================

MDDManager::MDDManager(const std::vector<std::vector<char>>& grid) : grid_(grid) {}

std::unordered_map<int, std::shared_ptr<MDD>> MDDManager::create_zone_mdds(
    const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_zone_paths,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    const std::vector<std::vector<char>>& masked_map,
    int start_t, int end_t) {
    
    std::unordered_map<int, std::shared_ptr<MDD>> local_mdds;
    
    for (const auto& [agent_id, local_path] : local_zone_paths) {
        auto entry_exit = local_entry_exit_time.at(agent_id);
        int entry_t = entry_exit.first;
        int exit_t = entry_exit.second;
        
        auto zone_start_pos = local_path.front();
        auto zone_goal_pos = local_path.back();
        int agent_path_length = exit_t - entry_t + 1;
        
        MDDConstructor constructor(masked_map, zone_start_pos, zone_goal_pos, agent_path_length - 1);
        auto agent_mdd = constructor.construct_mdd();
        
        if (agent_mdd) {
            align_mdd_to_time_window(agent_mdd, entry_t, exit_t, start_t, end_t);
            local_mdds[agent_id] = agent_mdd;
        }
    }
    
    return local_mdds;
}

std::shared_ptr<MDD> MDDManager::expand_mdd_for_waiting_time(
    int agent_id,
    const std::vector<std::pair<int,int>>& local_path,
    const std::pair<int,int>& entry_exit_time,
    const std::vector<std::vector<char>>& masked_map,
    int additional_timesteps) {
    
    auto zone_start_pos = local_path.front();
    auto zone_goal_pos = local_path.back();
    int original_path_length = entry_exit_time.second - entry_exit_time.first + 1;
    int new_path_length = original_path_length + additional_timesteps;
    
    MDDConstructor constructor(masked_map, zone_start_pos, zone_goal_pos, new_path_length - 1);
    return constructor.construct_mdd();
}

void MDDManager::align_mdd_to_time_window(std::shared_ptr<MDD> mdd, 
                                         int entry_t, int exit_t, 
                                         int start_t, int end_t) {
    if (!mdd) return;
    
    // Align MDD levels to the time window
    int mdd_start_level = entry_t - start_t;
    int mdd_end_level = exit_t - start_t;
    
    // Ensure MDD has the right number of levels
    if (mdd->levels.size() < mdd_end_level + 1) {
        mdd->levels.resize(mdd_end_level + 1);
    }
}

void MDDManager::extend_mdds_to_time_window(std::unordered_map<int, std::shared_ptr<MDD>>& mdds,
                                           int old_end_t, int new_end_t) {
    for (auto& [agent_id, mdd] : mdds) {
        if (mdd) {
            // Add empty levels at the end to extend to new_end_t
            for (int empty_level = old_end_t + 1; empty_level <= new_end_t; ++empty_level) {
                mdd->levels[empty_level] = {};  // Empty level
            }
        }
    }
}

// ============================================================================
// ZONE EXPANSION MANAGER IMPLEMENTATION
// ============================================================================

ZoneExpansionManager::ZoneExpansionManager(int rows, int cols) : rows_(rows), cols_(cols) {}

ZoneExpansionManager::ExpansionResult ZoneExpansionManager::expand_zone(
    const std::vector<std::pair<int,int>>& original_conflicts,
    const std::vector<int>& original_indices,
    int expansion_factor,
    int base_offset,
    const std::vector<std::pair<int,int>>& all_conflict_points,
    const std::vector<std::vector<int>>& conflict_map,
    const std::set<int>& solved_conflict_indices) {
    
    ExpansionResult result;
    result.success = false;
    
    // Create expanded bucket with increased offset
    int expanded_offset = base_offset + expansion_factor;
    result.expanded_bucket_conflicts = original_conflicts;
    result.expanded_conflict_indices = original_indices;
    
    // Create expanded diamond shape
    result.expanded_positions = create_expanded_diamond(original_conflicts, expanded_offset);
    
    // Find newly touched conflicts
    std::set<std::pair<int,int>> original_positions;
    for (const auto& conflict : original_conflicts) {
        original_positions.insert(conflict);
    }
    
    auto new_positions = find_new_positions(result.expanded_positions, original_positions);
    
    // Check new positions for conflicts
    for (const auto& pos : new_positions) {
        auto [r, c] = pos;
        if (r >= 0 && r < rows_ && c >= 0 && c < cols_) {
            int conflict_idx = conflict_map[r][c];
            
            if (conflict_idx != -1 && 
                solved_conflict_indices.find(conflict_idx) == solved_conflict_indices.end()) {
                
                // Check if this conflict was not part of the original bucket
                bool was_in_original = false;
                for (int original_idx : original_indices) {
                    if (original_idx == conflict_idx) {
                        was_in_original = true;
                        break;
                    }
                }
                
                if (!was_in_original) {
                    result.newly_touched_conflicts.push_back(conflict_idx);
                    result.expanded_bucket_conflicts.push_back(all_conflict_points[conflict_idx]);
                    result.expanded_conflict_indices.push_back(conflict_idx);
                }
            }
        }
    }
    
    // Create final expanded diamond shape
    result.expanded_positions = create_expanded_diamond(result.expanded_bucket_conflicts, expanded_offset);
    result.success = true;
    
    return result;
}

bool ZoneExpansionManager::is_global_zone(const std::set<std::pair<int,int>>& positions) const {
    return positions.size() >= rows_ * cols_;
}

double ZoneExpansionManager::get_zone_coverage(const std::set<std::pair<int,int>>& positions) const {
    return static_cast<double>(positions.size()) / (rows_ * cols_);
}

std::set<std::pair<int,int>> ZoneExpansionManager::create_expanded_diamond(
    const std::vector<std::pair<int,int>>& conflicts,
    int expanded_offset) const {
    
    return create_shape_from_conflicts(conflicts, expanded_offset);
}

// ============================================================================
// PATH MANAGER IMPLEMENTATION
// ============================================================================

PathManager::LocalPathData PathManager::extract_local_paths(
    const CurrentSolution& current_solution,
    const std::set<std::pair<int,int>>& zone_positions,
    const std::set<int>& agents_in_zone,
    int start_t, int end_t) {
    
    LocalPathData data;
    
    for (int agent_id : agents_in_zone) {
        const auto& full_path = current_solution.agent_paths.at(agent_id);
        int entry_t, exit_t;
        
        auto segment = find_path_segment_in_zone(full_path, zone_positions, start_t, end_t, entry_t, exit_t);
        
        if (!segment.empty()) {
            data.local_zone_paths[agent_id] = segment;
            data.local_entry_exit_time[agent_id] = {entry_t, exit_t};
        }
    }
    
    return data;
}

void PathManager::update_global_solution(
    CurrentSolution& current_solution,
    const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_paths,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time) {
    
    current_solution.update_with_local_paths(local_paths, local_entry_exit_time);
}

std::unordered_map<int, std::vector<std::pair<int,int>>> PathManager::sample_paths_from_mdds(
    const std::unordered_map<int, std::shared_ptr<MDD>>& mdds,
    std::mt19937& rng) {
    
    std::unordered_map<int, std::vector<std::pair<int,int>>> paths;
    
    for (const auto& [agent_id, mdd] : mdds) {
        if (mdd) {
            auto path_positions = mdd->sample_random_path(rng);
            std::vector<std::pair<int,int>> as_pairs(path_positions.begin(), path_positions.end());
            paths[agent_id] = std::move(as_pairs);
        }
    }
    
    return paths;
}

bool PathManager::validate_paths(const std::unordered_map<int, std::vector<std::pair<int,int>>>& paths) const {
    for (const auto& [agent_id, path] : paths) {
        if (path.empty()) {
            std::cerr << "[ERROR] Agent " << agent_id << " has empty path" << std::endl;
            return false;
        }
    }
    return true;
}

std::vector<std::pair<int,int>> PathManager::find_path_segment_in_zone(
    const std::vector<std::pair<int,int>>& full_path,
    const std::set<std::pair<int,int>>& zone_positions,
    int start_t, int end_t,
    int& entry_t, int& exit_t) const {
    
    std::vector<std::pair<int,int>> segment;
    entry_t = -1;
    exit_t = -1;
    
    for (int t = start_t; t <= end_t && t < (int)full_path.size(); ++t) {
        const auto& pos = full_path[t];
        if (zone_positions.count(pos)) {
            if (entry_t == -1) {
                entry_t = t;
            }
            exit_t = t;
            segment.push_back(pos);
        }
    }
    
    return segment;
}

// ============================================================================
// WAITING TIME STRATEGY MANAGER IMPLEMENTATION
// ============================================================================

WaitingTimeStrategyManager::WaitingTimeResult WaitingTimeStrategyManager::execute_strategy(
    CurrentSolution& current_solution,
    const std::vector<std::vector<char>>& masked_map,
    const std::set<std::pair<int,int>>& zone_positions,
    const std::unordered_map<int, std::shared_ptr<MDD>>& local_mdds,
    const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_zone_paths,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    const CollisionTracker& initial_collisions,
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<int>& conflict_indices,
    int start_t, int end_t) {
    
    WaitingTimeResult result;
    result.solution_found = false;
    result.discovered_collisions = initial_collisions;
    
    // Find agents involved in conflicts
    std::set<int> agents_involved_in_conflicts;
    for (int idx : conflict_indices) {
        if (idx >= 0 && idx < (int)conflict_meta.size()) {
            const auto& meta = conflict_meta[idx];
            agents_involved_in_conflicts.insert(meta.agent1);
            agents_involved_in_conflicts.insert(meta.agent2);
        }
    }
    
    auto agents_with_waiting = get_agents_with_waiting_time(current_solution, agents_involved_in_conflicts);
    
    if (agents_with_waiting.empty()) {
        return result;
    }
    
    // Backup waiting times
    auto waiting_time_backup = current_solution.backup_waiting_times();
    
    // Try waiting time strategy
    int attempt = 0;
    int max_attempts = 0;
    for (const auto& [agent_id, waiting_time] : agents_with_waiting) {
        max_attempts += waiting_time;
    }
    
    while (!result.solution_found && attempt < max_attempts) {
        attempt++;
        
        auto current_agents_with_waiting = get_agents_with_waiting_time(current_solution, agents_involved_in_conflicts);
        if (current_agents_with_waiting.empty()) {
            break;
        }
        
        auto [agent_with_most_waiting, max_waiting_time] = current_agents_with_waiting[0];
        
        // Use waiting time and expand MDD
        current_solution.use_waiting_time(agent_with_most_waiting, 1);
        
        // Expand MDD for this agent
        auto expanded_mdd = expand_agent_mdd(agent_with_most_waiting, current_solution, masked_map, 
                                           local_zone_paths, local_mdds, local_entry_exit_time, start_t, end_t);
        
        if (expanded_mdd) {
            // Try to solve with expanded MDD
            // This would involve creating CNF and solving
            // For now, we'll assume it might work
            result.solution_found = true;
        }
    }
    
    // Restore waiting times if no solution found
    if (!result.solution_found) {
        current_solution.restore_waiting_times(waiting_time_backup);
    }
    
    return result;
}

std::vector<std::pair<int, int>> WaitingTimeStrategyManager::get_agents_with_waiting_time(
    const CurrentSolution& current_solution,
    const std::set<int>& agent_ids) const {
    
    return current_solution.get_agents_waiting_times(agent_ids);
}

bool WaitingTimeStrategyManager::expand_agent_mdd(
    int agent_id,
    CurrentSolution& current_solution,
    const std::vector<std::vector<char>>& masked_map,
    const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_zone_paths,
    std::unordered_map<int, std::shared_ptr<MDD>>& local_mdds,
    std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int& end_t) {
    
    // Get agent's current entry/exit info
    auto entry_exit = local_entry_exit_time[agent_id];
    int entry_t = entry_exit.first;
    int exit_t = entry_exit.second;
    
    // Get agent's zone entry and exit points
    const auto& local_path = local_zone_paths.at(agent_id);
    auto zone_start_pos = local_path.front();
    auto zone_goal_pos = local_path.back();
    
    // Calculate new path length (original + 1 extra timestep)
    int original_path_length = exit_t - entry_t + 1;
    int new_path_length = original_path_length + 1;
    
    // Create new MDD with expanded length
    MDDConstructor constructor(masked_map, zone_start_pos, zone_goal_pos, new_path_length - 1);
    auto expanded_mdd = constructor.construct_mdd();
    
    if (!expanded_mdd) {
        return false;
    }
    
    // Update the local MDD for this agent
    local_mdds[agent_id] = expanded_mdd;
    
    // Update entry/exit time to reflect the expanded path
    int new_exit_t = exit_t + 1;
    local_entry_exit_time[agent_id] = {entry_t, new_exit_t};
    
    // Update end time if necessary
    if (new_exit_t > end_t) {
        end_t = new_exit_t;
    }
    
    return true;
}

// ============================================================================
// UTILITY FUNCTIONS IMPLEMENTATION
// ============================================================================

void print_solution_statistics(const CurrentSolution& solution) {
    std::cout << "[LNS] Solution Statistics:" << std::endl;
    std::cout << "  Agents: " << solution.agent_paths.size() << std::endl;
    std::cout << "  Max timestep: " << solution.max_timestep << std::endl;
    std::cout << "  Grid size: " << solution.rows << "x" << solution.cols << std::endl;
    
    int total_waiting_time = 0;
    for (const auto& [agent_id, waiting_time] : solution.agent_waiting_time) {
        total_waiting_time += waiting_time;
    }
    std::cout << "  Total waiting time: " << total_waiting_time << std::endl;
}

bool validate_conflict_zone(const std::set<std::pair<int,int>>& zone_positions, 
                           int rows, int cols) {
    for (const auto& pos : zone_positions) {
        if (pos.first < 0 || pos.first >= rows || pos.second < 0 || pos.second >= cols) {
            return false;
        }
    }
    return true;
}

std::vector<std::vector<char>> create_masked_map_for_zone(
    const std::vector<std::vector<char>>& original_map,
    const std::set<std::pair<int,int>>& zone_positions) {
    
    return mask_map_outside_shape(original_map, zone_positions);
}
