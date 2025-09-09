#include "LNSHelpers.h"
#include "../SATSolverManager.h"
#include <iostream>
#include <random>

// Example demonstrating how to use the refactored LNS helper classes
int main() {
    std::cout << "=== LNS Helper Classes Usage Example ===" << std::endl;
    
    // Example 1: Using CollisionTracker
    std::cout << "\n1. CollisionTracker Example:" << std::endl;
    CollisionTracker tracker;
    
    // Add some example collisions
    std::vector<CollisionTracker::VertexCollision> vertex_collisions = {
        {0, 1, {5, 5}, 10},  // Agent 0 and 1 collide at position (5,5) at time 10
        {2, 3, {7, 8}, 15}   // Agent 2 and 3 collide at position (7,8) at time 15
    };
    
    std::vector<CollisionTracker::EdgeCollision> edge_collisions = {
        {0, 2, {3, 4}, {4, 4}, 12}  // Agent 0 and 2 swap positions at time 12
    };
    
    tracker.add_vertex_collisions(vertex_collisions);
    tracker.add_edge_collisions(edge_collisions);
    tracker.print_summary("  ");
    
    // Example 2: Using ConflictBucketManager
    std::cout << "\n2. ConflictBucketManager Example:" << std::endl;
    int rows = 20, cols = 20, offset = 2;
    ConflictBucketManager bucket_manager(rows, cols, offset);
    
    // Create some example conflict points
    std::vector<std::pair<int,int>> conflict_points = {
        {5, 5}, {7, 8}, {3, 4}, {4, 4}
    };
    
    // Create conflict metadata
    std::vector<ConflictMeta> conflict_meta = {
        {0, 1, 10, false, {5, 5}, {-1, -1}},  // Vertex conflict
        {2, 3, 15, false, {7, 8}, {-1, -1}},  // Vertex conflict
        {0, 2, 12, true, {3, 4}, {4, 4}},     // Edge conflict
        {0, 2, 12, true, {3, 4}, {4, 4}}      // Edge conflict (both endpoints)
    };
    
    std::set<int> solved_conflict_indices; // Empty for this example
    
    auto buckets = bucket_manager.create_diamond_buckets(
        conflict_points, conflict_meta, solved_conflict_indices);
    
    bucket_manager.print_bucket_statistics(buckets);
    
    // Example 3: Using ZoneExpansionManager
    std::cout << "\n3. ZoneExpansionManager Example:" << std::endl;
    ZoneExpansionManager expansion_manager(rows, cols);
    
    // Create conflict map for efficient queries
    auto conflict_map = bucket_manager.create_conflict_map(conflict_points);
    
    // Try expanding a zone
    std::vector<std::pair<int,int>> original_conflicts = {{5, 5}, {7, 8}};
    std::vector<int> original_indices = {0, 1};
    int expansion_factor = 1;
    int base_offset = 2;
    
    auto expansion_result = expansion_manager.expand_zone(
        original_conflicts, original_indices, expansion_factor, base_offset,
        conflict_points, conflict_map, solved_conflict_indices);
    
    if (expansion_result.success) {
        std::cout << "  Zone expansion successful!" << std::endl;
        std::cout << "  Expanded positions: " << expansion_result.expanded_positions.size() << std::endl;
        std::cout << "  Newly touched conflicts: " << expansion_result.newly_touched_conflicts.size() << std::endl;
        
        double coverage = expansion_manager.get_zone_coverage(expansion_result.expanded_positions);
        std::cout << "  Zone coverage: " << (coverage * 100.0) << "%" << std::endl;
    } else {
        std::cout << "  Zone expansion failed" << std::endl;
    }
    
    // Example 4: Using PathManager
    std::cout << "\n4. PathManager Example:" << std::endl;
    PathManager path_manager;
    
    // Create a simple example solution
    CurrentSolution example_solution(rows, cols, 20, 4);
    
    // Add some example paths
    example_solution.agent_paths[0] = {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};
    example_solution.agent_paths[1] = {{0, 1}, {1, 1}, {2, 1}, {3, 1}, {4, 1}};
    example_solution.agent_paths[2] = {{0, 2}, {1, 2}, {2, 2}, {3, 2}, {4, 2}};
    example_solution.agent_paths[3] = {{0, 3}, {1, 3}, {2, 3}, {3, 3}, {4, 3}};
    
    // Create a conflict zone
    std::set<std::pair<int,int>> zone_positions = {
        {2, 0}, {2, 1}, {2, 2}, {2, 3}, {3, 0}, {3, 1}, {3, 2}, {3, 3}
    };
    
    // Find agents in zone
    std::set<int> agents_in_zone = {0, 1, 2, 3};
    
    // Extract local paths
    auto local_path_data = path_manager.extract_local_paths(
        example_solution, zone_positions, agents_in_zone, 0, 20);
    
    std::cout << "  Extracted local paths for " << local_path_data.local_zone_paths.size() << " agents" << std::endl;
    
    // Validate paths
    bool valid = path_manager.validate_paths(example_solution.agent_paths);
    std::cout << "  Path validation: " << (valid ? "PASSED" : "FAILED") << std::endl;
    
    // Example 5: Using WaitingTimeStrategyManager
    std::cout << "\n5. WaitingTimeStrategyManager Example:" << std::endl;
    WaitingTimeStrategyManager waiting_time_manager;
    
    // Calculate waiting times for the example solution
    std::vector<std::pair<int,int>> goals = {{4, 0}, {4, 1}, {4, 2}, {4, 3}};
    example_solution.calculate_waiting_times(goals, 20);
    
    // Get agents with waiting time
    auto agents_with_waiting = waiting_time_manager.get_agents_with_waiting_time(
        example_solution, agents_in_zone);
    
    std::cout << "  Agents with waiting time: " << agents_with_waiting.size() << std::endl;
    for (const auto& [agent_id, waiting_time] : agents_with_waiting) {
        std::cout << "    Agent " << agent_id << ": " << waiting_time << " timesteps" << std::endl;
    }
    
    // Example 6: Using MDDManager
    std::cout << "\n6. MDDManager Example:" << std::endl;
    
    // Create a simple grid
    std::vector<std::vector<char>> grid(rows, std::vector<char>(cols, '.'));
    // Add some obstacles
    for (int i = 0; i < rows; ++i) {
        grid[i][0] = grid[i][cols-1] = '@';  // Vertical walls
    }
    for (int j = 0; j < cols; ++j) {
        grid[0][j] = grid[rows-1][j] = '@';  // Horizontal walls
    }
    
    MDDManager mdd_manager(grid);
    std::cout << "  MDDManager initialized with " << rows << "x" << cols << " grid" << std::endl;
    
    // Example 7: Utility Functions
    std::cout << "\n7. Utility Functions Example:" << std::endl;
    
    // Print solution statistics
    print_solution_statistics(example_solution);
    
    // Validate conflict zone
    bool zone_valid = validate_conflict_zone(zone_positions, rows, cols);
    std::cout << "  Conflict zone validation: " << (zone_valid ? "PASSED" : "FAILED") << std::endl;
    
    // Create masked map
    auto masked_map = create_masked_map_for_zone(grid, zone_positions);
    std::cout << "  Created masked map with " << masked_map.size() << "x" << masked_map[0].size() << " dimensions" << std::endl;
    
    std::cout << "\n=== Example Complete ===" << std::endl;
    
    return 0;
}
