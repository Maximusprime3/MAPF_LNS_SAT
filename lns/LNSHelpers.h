#pragma once

#include "../SATSolverManager.h"
#include "LNSCore.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <set>
#include <tuple>
#include <memory>

// Core types are defined in LNSCore.h

// ============================================================================
// COLLISION TRACKER - Manages collision discovery and tracking
// ============================================================================
class CollisionTracker {
public:
    // Collision types
    using VertexCollision = std::tuple<int, int, std::pair<int,int>, int>;
    using EdgeCollision = std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>;
    
    // Constructor
    CollisionTracker() = default;
    
    // Add collisions from conflict detection
    void add_vertex_collisions(const std::vector<VertexCollision>& collisions);
    void add_edge_collisions(const std::vector<EdgeCollision>& collisions);
    
    // Get all discovered collisions
    const std::vector<VertexCollision>& get_vertex_collisions() const { return vertex_collisions_; }
    const std::vector<EdgeCollision>& get_edge_collisions() const { return edge_collisions_; }
    
    // Merge collisions from another tracker
    void merge_collisions(const CollisionTracker& other);
    
    // Clear all collisions
    void clear();
    
    // Get collision counts
    size_t get_vertex_count() const { return vertex_collisions_.size(); }
    size_t get_edge_count() const { return edge_collisions_.size(); }
    size_t get_total_count() const { return vertex_collisions_.size() + edge_collisions_.size(); }
    
    // Print collision summary
    void print_summary(const std::string& prefix = "") const;

private:
    std::vector<VertexCollision> vertex_collisions_;
    std::vector<EdgeCollision> edge_collisions_;
};

// ============================================================================
// CONFLICT BUCKET MANAGER - Handles conflict zone creation and management
// ============================================================================
class ConflictBucketManager {
public:
    // Constructor
    ConflictBucketManager(int rows, int cols,
        const std::vector<std::vector<char>>& grid,
        int offset = 2);
    
    // Create conflict buckets from collision data
    std::vector<DiamondBucket> create_diamond_buckets(
        const std::vector<std::pair<int,int>>& conflict_points,
        const std::vector<ConflictMeta>& conflict_meta,
        const std::set<int>& solved_conflict_indices);
    
    // Find conflicts in new positions (for zone expansion)
    std::vector<int> find_conflicts_in_positions(
        const std::set<std::pair<int,int>>& positions,
        const std::vector<std::pair<int,int>>& conflict_points,
        const std::set<int>& solved_conflict_indices,
        const std::set<int>& original_bucket_indices);
    
    // Create conflict map for efficient spatial queries
    std::vector<std::vector<int>> create_conflict_map(
        const std::vector<std::pair<int,int>>& conflict_points);
    
    // Get bucket statistics
    void print_bucket_statistics(const std::vector<DiamondBucket>& buckets) const;

private:
    std::vector<std::vector<char>>& grid_;
    int rows_, cols_, offset_;
    
    // Helper: check if position is within diamond shape
    bool is_within_diamond(const std::pair<int,int>& pos, 
                          const std::pair<int,int>& center, 
                          int radius) const;
    
    // Helper: find new positions in expanded shape
    std::set<std::pair<int,int>> find_new_positions(
        const std::set<std::pair<int,int>>& expanded_shape,
        const std::set<std::pair<int,int>>& original_shape) const;
};

// ============================================================================
// MDD MANAGER - Handles MDD construction and management
// ============================================================================
class MDDManager {
public:
    // Constructor
    MDDManager(const std::vector<std::vector<char>>& grid);
    
    // Create MDDs for agents in a conflict zone
    std::unordered_map<int, std::shared_ptr<MDD>> create_zone_mdds(
        const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_zone_paths,
        const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
        const std::vector<std::vector<char>>& masked_map,
        int start_t, int end_t);
    
    // Expand MDD for waiting time strategy
    std::shared_ptr<MDD> expand_mdd_for_waiting_time(
        int agent_id,
        const std::vector<std::pair<int,int>>& local_path,
        const std::pair<int,int>& entry_exit_time,
        const std::vector<std::vector<char>>& masked_map,
        int additional_timesteps = 1);
    
    // Align MDD to time window
    void align_mdd_to_time_window(std::shared_ptr<MDD> mdd, 
                                 int entry_t, int exit_t, 
                                 int start_t, int end_t);
    
    // Extend existing MDDs to new time window
    void extend_mdds_to_time_window(std::unordered_map<int, std::shared_ptr<MDD>>& mdds,
                                   int old_end_t, int new_end_t);

private:
    const std::vector<std::vector<char>>& grid_;
};

// ============================================================================
// ZONE EXPANSION MANAGER - Handles conflict zone expansion logic
// ============================================================================
class ZoneExpansionManager {
public:
    // Constructor
    ZoneExpansionManager(int rows, int cols);
    
    // Expand conflict zone with given factor
    struct ExpansionResult {
        bool success;
        std::set<std::pair<int,int>> expanded_positions;
        std::vector<int> newly_touched_conflicts;
        std::vector<std::pair<int,int>> expanded_bucket_conflicts;
        std::vector<int> expanded_conflict_indices;
    };
    
    ExpansionResult expand_zone(
        const std::vector<std::pair<int,int>>& original_conflicts,
        const std::vector<int>& original_indices,
        int expansion_factor,
        int base_offset,
        const std::vector<std::pair<int,int>>& all_conflict_points,
        const std::vector<std::vector<int>>& conflict_map,
        const std::set<int>& solved_conflict_indices);
    
    // Check if zone covers entire map
    bool is_global_zone(const std::set<std::pair<int,int>>& positions) const;
    
    // Get zone coverage percentage
    double get_zone_coverage(const std::set<std::pair<int,int>>& positions) const;

private:
    int rows_, cols_;
    
    // Helper: create expanded diamond shape
    std::set<std::pair<int,int>> create_expanded_diamond(
        const std::vector<std::pair<int,int>>& conflicts,
        int expanded_offset) const;
};

// ============================================================================
// PATH MANAGER - Handles path operations and solution updates
// ============================================================================
class PathManager {
public:
    // Constructor
    PathManager() = default;
    
    // Extract local paths for agents in conflict zone
    struct LocalPathData {
        std::unordered_map<int, std::vector<std::pair<int,int>>> local_zone_paths;
        std::unordered_map<int, std::pair<int,int>> local_entry_exit_time;
    };
    
    LocalPathData extract_local_paths(
        const CurrentSolution& current_solution,
        const std::set<std::pair<int,int>>& zone_positions,
        const std::set<int>& agents_in_zone,
        int start_t, int end_t);
    
    // Update global solution with local paths
    void update_global_solution(
        CurrentSolution& current_solution,
        const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_paths,
        const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time);
    
    // Sample paths from MDDs
    std::unordered_map<int, std::vector<std::pair<int,int>>> sample_paths_from_mdds(
        const std::unordered_map<int, std::shared_ptr<MDD>>& mdds,
        std::mt19937& rng);
    
    // Validate path consistency
    bool validate_paths(const std::unordered_map<int, std::vector<std::pair<int,int>>>& paths) const;

private:
    // Helper: find path segment in zone
    std::vector<std::pair<int,int>> find_path_segment_in_zone(
        const std::vector<std::pair<int,int>>& full_path,
        const std::set<std::pair<int,int>>& zone_positions,
        int start_t, int end_t,
        int& entry_t, int& exit_t) const;
};

// ============================================================================
// WAITING TIME STRATEGY MANAGER - Handles waiting time conflict resolution
// ============================================================================
class WaitingTimeStrategyManager {
public:
    // Constructor
    WaitingTimeStrategyManager() = default;
    
    // Execute waiting time strategy
    struct WaitingTimeResult {
        bool solution_found;
        std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths;
        CollisionTracker discovered_collisions;
    };
    
    WaitingTimeResult execute_strategy(
        CurrentSolution& current_solution,
        const std::vector<std::vector<char>>& masked_map,
        const std::set<std::pair<int,int>>& zone_positions,
        const std::unordered_map<int, std::shared_ptr<MDD>>& local_mdds,
        const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_zone_paths,
        const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
        const CollisionTracker& initial_collisions,
        const std::vector<ConflictMeta>& conflict_meta,
        const std::vector<int>& conflict_indices,
        int start_t, int end_t);
    
    // Get agents with waiting time
    std::vector<std::pair<int, int>> get_agents_with_waiting_time(
        const CurrentSolution& current_solution,
        const std::set<int>& agent_ids) const;

private:
    // Helper: expand MDD for agent with waiting time
    bool expand_agent_mdd(
        int agent_id,
        CurrentSolution& current_solution,
        const std::vector<std::vector<char>>& masked_map,
        const std::unordered_map<int, std::vector<std::pair<int,int>>>& local_zone_paths,
        std::unordered_map<int, std::shared_ptr<MDD>>& local_mdds,
        std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
        int start_t, int& end_t);
};

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

// Print solution statistics
void print_solution_statistics(const CurrentSolution& solution);

// Validate conflict zone
bool validate_conflict_zone(const std::set<std::pair<int,int>>& zone_positions, 
                           int rows, int cols);

// Create masked map for zone
std::vector<std::vector<char>> create_masked_map_for_zone(
    const std::vector<std::vector<char>>& original_map,
    const std::set<std::pair<int,int>>& zone_positions);
