#pragma once

#include "Current_Solution.h"
#include <vector>
#include <set>
#include <utility>

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

// Represents the result of extracting collisions from a bucket
struct CollisionExtractionResult {
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> edge_collisions;
    int vertex_count;
    int edge_count;
};


/**
 * @brief Mask the map outside a given shape with '@'.
 * @param map  2D grid of map characters.
 * @param shape_positions  Set of positions to keep from the map.
 * @return Masked map where only shape cells mirror the input map.
 */
std::vector<std::vector<char>> mask_map_outside_shape(
    const std::vector<std::vector<char>>& map,
    const std::set<std::pair<int,int>>& shape_positions);

/**
 * @brief Union of diamond regions (Manhattan radius) around conflicts.
 * @param conflict_points  Conflict coordinates (row,col).
 * @param expansion_radius Diamond Manhattan radius (>=0).
 * @param map  Used for bounds and walkability filtering.
 * @return Set of positions in the union of valid diamond cells.
 */
std::set<std::pair<int,int>> create_shape_from_conflicts(
    const std::vector<std::pair<int,int>>& conflict_points,
    int expansion_radius,
    const std::vector<std::vector<char>>& map);

/**
 * @brief New positions in current shape vs previous, bounded to walkable map.
 * @param current_shape  Current set of positions.
 * @param previous_shape Previous set of positions.
 * @param map  Used for bounds and walkability filtering.
 * @return Set difference filtered to in-bounds walkable cells.
 */
std::set<std::pair<int,int>> find_new_positions(
    const std::set<std::pair<int,int>>& current_shape,
    const std::set<std::pair<int,int>>& previous_shape,
    const std::vector<std::vector<char>>& map);

/**
 * @brief Group conflicts into disjoint diamond buckets via iterative growth.
 * @param building_conflicts_meta  Per-conflict metadata (timestep, agents, position(s)).
 * @param original_building_conflict_indices  Global indices of the building conflicts.
 * @param conflict_map  (r,c) -> list of conflict indices.
 * @param map  Used for bounds/walkability of shapes.
 * @param all_conflict_meta  Per-conflict metadata (timestep, agents, etc.).
 * @param offset  Diamond radius used during grouping.
 * @param solved_conflict_indices  Conflicts to exclude (default empty set).
 * @return Vector of buckets with positions, indices, and time window.
 */
std::vector<DiamondBucket> build_diamond_buckets(
    const std::vector<ConflictMeta>& building_conflicts_meta,
    const std::set<int>& original_building_conflict_indices,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<std::vector<char>>& map,
    const std::vector<ConflictMeta>& all_conflict_meta,
    int offset,
    const std::set<int>& solved_conflict_indices = std::set<int>());
/**
 * @brief Group conflicts into disjoint diamond buckets via iterative growth for earliest conflicts.
 * @param all_conflict_meta  Per-conflict metadata (timestep, agents, etc.).
 * @param conflict_map  (r,c) -> list of conflict indices.
 * @param map  grid map used for bounds/walkability of shapes.
 * @param offset  Diamond radius used during grouping.
 * @return Vector of buckets with positions, indices, and time window for earliest conflicts.
 */
std::vector<DiamondBucket> build_diamond_buckets_for_earliest_conflicts(
    const std::vector<ConflictMeta>& all_conflict_meta,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<std::vector<char>>& map,
    int offset);

/**
 * @brief Expand a bucket's zone with a larger offset and absorb new conflicts.
 * @param conflict_meta  Per-conflict metadata.
 * @param conflict_map  (r,c) -> list of conflict indices.
 * @param map  Used for bounds/walkability of shapes.
 * @param original_bucket_indices  Global indices of the original bucket.
 * @param original_bucket_positions Positions of the original bucket.
 * @param expanded_offset  New diamond radius.
 * @return {expanded positions set, full list of conflict indices}.
 */
std::pair<std::set<std::pair<int,int>>, std::vector<int>> expand_bucket_zone(
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<std::vector<char>>& map,
    const std::vector<int>& original_bucket_indices,
    const std::set<std::pair<int,int>>& original_bucket_positions,
    int expanded_offset);

/**
 * @brief Extract vertex and edge collisions from conflict indices.
 * @param bucket_indices  Conflict indices belonging to a bucket.
 * @param conflict_meta  Metadata array indexed by conflict index.
 * @return Aggregated collision result with counts and lists.
 */
CollisionExtractionResult extract_collisions_from_bucket(
    const std::vector<int>& bucket_indices,
    const std::vector<ConflictMeta>& conflict_meta);

/**
 * @brief Validate that buckets form a disjoint cover over conflicts.
 * @param conflict_points  All conflict positions (size = num conflicts).
 * @param diamond_buckets  Buckets with their member conflict indices.
 * @return true if every conflict index appears in exactly one bucket; false otherwise.
 */
bool validate_buckets_cover_conflicts(
    const std::vector<std::pair<int,int>>& conflict_points,
    const std::vector<DiamondBucket>& diamond_buckets);

/**
 * @brief Select the most relevant bucket based on conflict count, then shape size.
 * @param diamond_buckets  Buckets to evaluate.
 * @return The selected bucket, or an empty bucket if none.
 */
DiamondBucket select_most_relevant_bucket(const std::vector<DiamondBucket>& diamond_buckets);