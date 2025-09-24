#include "Local_Zone.h"
#include "Current_Solution.h"

#include <cmath>
#include <algorithm>  // for std::set_difference
#include <iterator>   // for std::inserter

/**
 * @brief Mask a map outside a provided shape.
 *
 * Copies the characters of \p map only at positions listed in \p shape_positions
 * and fills every other cell with the obstacle character '@'.
 *
 * @param map              2D grid of map characters (rows x cols).
 * @param shape_positions  Set of (row,col) coordinates to preserve from \p map.
 * @return A 2D grid where cells outside \p shape_positions are '@', and cells
 *         inside the shape mirror the original \p map values. If \p map is empty,
 *         returns an empty grid.
 *
 * @pre Coordinates in \p shape_positions use the same row/col convention as \p map.
 * @post Return grid dimensions match \p map.
 * @note No side effects.
 * @complexity O(rows*cols + |shape_positions|), dominated by grid initialization.
 */
std::vector<std::vector<char>> mask_map_outside_shape(
    const std::vector<std::vector<char>>& map,
    const std::set<std::pair<int,int>>& shape_positions) {
    if (map.empty() || map[0].empty()) return {};
    int rows = static_cast<int>(map.size());
    int cols = static_cast<int>(map[0].size());

    // Start with a map filled with obstacles and carve out the shape.
    std::vector<std::vector<char>> masked(rows, std::vector<char>(cols, '@'));

    for (const auto& pos : shape_positions) {
        int r = pos.first;
        int c = pos.second;
        if (r >= 0 && r < rows && c >= 0 && c < cols) {
            masked[r][c] = map[r][c];
        }
    }

    return masked;
}


/**
 * @brief Build a diamond-shaped region around conflict points.
 *
 * For each conflict point, includes all cells within Manhattan distance
 * \p expansion_radius, filtered to walkable ('.') or goal ('G') cells
 * within \p map bounds. The union over all conflicts is returned.
 *
 * @param conflict_points   List of (row,col) conflict coordinates.
 * @param expansion_radius  Non-negative Manhattan radius for the diamond shape.
 * @param map               2D grid used for bounds and walkability checks.
 * @return Set of positions comprising the union of per-conflict diamonds.
 *
 * @pre 0 <= expansion_radius. Coordinates refer to rows and columns of \p map.
 * @post Returned positions are within bounds and map cells are either '.' or 'G'.
 * @note Duplicates are naturally removed by the set container.
 * @complexity O(|conflict_points| * expansion_radius^2).
 */
std::set<std::pair<int,int>> create_shape_from_conflicts(
    const std::vector<std::pair<int,int>>& conflict_points,
    int expansion_radius,
    const std::vector<std::vector<char>>& map) {
    int rows = (int)map.size();
    int cols = rows > 0 ? (int)map[0].size() : 0;
    std::set<std::pair<int,int>> shape_positions;

    for (const auto& conflict_point : conflict_points) {
        int center_row = conflict_point.first;
        int center_col = conflict_point.second;

        for (int r = center_row - expansion_radius; r <= center_row + expansion_radius; ++r) {
            for (int c = center_col - expansion_radius; c <= center_col + expansion_radius; ++c) {
                if (r < 0 || r >= rows || c < 0 || c >= cols) continue;
                if (std::abs(r - center_row) + std::abs(c - center_col) <= expansion_radius) {
                    // Only include walkable ('.') or goal ('G') cells
                    char cell = map[r][c];
                    if (cell == '.' || cell == 'G') {
                        shape_positions.insert({r, c});
                    }
                }
            }
        }
    }

    return shape_positions;
}


/**
 * @brief Compute newly added positions between two shapes, bounded to walkable map cells.
 *
 * Computes the set difference current_shape \ previous_shape, then filters
 * positions to be in-bounds and on walkable cells ('.' or 'G') according to \p map.
 *
 * @param current_shape    Set of (row,col) positions in the current shape.
 * @param previous_shape   Set of (row,col) positions in the previous shape.
 * @param map              2D grid used for bounds and walkability checks.
 * @return Set of new positions in current_shape that are valid and walkable.
 *
 * @pre Shapes use the same coordinate system as \p map.
 * @post All returned positions are within bounds and walkable.
 * @note No side effects. Stable under repeated calls.
 * @complexity O(|current_shape| log |current_shape| + |previous_shape| log |previous_shape|).
 */
std::set<std::pair<int,int>> find_new_positions(
    const std::set<std::pair<int,int>>& current_shape,
    const std::set<std::pair<int,int>>& previous_shape,
    const std::vector<std::vector<char>>& map) {
    int rows = (int)map.size();
    int cols = rows > 0 ? (int)map[0].size() : 0;
    // First compute the set difference
    auto diff = find_new_positions(current_shape, previous_shape);
    // Then filter by bounds and walkability
    std::set<std::pair<int,int>> bounded;
    for (const auto& pos : diff) {
        int r = pos.first, c = pos.second;
        if (r >= 0 && r < rows && c >= 0 && c < cols) {
            char cell = map[r][c];
            if (cell == '.' || cell == 'G') {
                bounded.insert(pos);
            }
        }
    }
    return bounded;
}


/**
 * @brief Group conflicts into disjoint diamond buckets by iterative growth.
 *
 * Starts from each unused conflict as a seed and grows a diamond of radius \p offset
 * repeatedly, merging conflicts that fall into newly added frontier cells. Each
 * conflict index is assigned to at most one bucket. Also tracks earliest/latest
 * timesteps within each bucket.
 *
 * @param conflict_points           Conflict grid positions (row,col) by index.
 * @param conflict_map              2D structure mapping (r,c) -> list of conflict indices.
 * @param map                       2D grid used for bounds/walkability of shapes.
 * @param solved_conflict_indices   Conflicts to exclude from new buckets.
 * @param conflict_meta             Per-conflict metadata (agents, timestep, edge/vertex, position(s)).
 * @param offset                    Diamond radius used for growth during grouping.
 * @return Vector of `DiamondBucket` describing positions, member indices and time window.
 *
 * @pre Dimensions of \p conflict_map match \p map.
 * @post Each returned bucket has non-empty `indices`; indices are unique across buckets.
 * @sideeffect Writes to stdout/stderr for tracing and anomaly reporting.
 * @complexity Empirically near-linear in number of conflicts plus touched frontier.
 */
std::vector<DiamondBucket> build_diamond_buckets(
    const std::vector<std::pair<int,int>>& conflict_points,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<std::vector<char>>& map,
    const std::vector<ConflictMeta>& conflict_meta,
    int offset,
    const std::set<int>& solved_conflict_indices) {

    std::vector<DiamondBucket> diamond_buckets;
    std::vector<char> diamond_used(conflict_points.size(), 0);
    // Dimensions for safe conflict_map access
    const int rows = (int)conflict_map.size();
    const int cols = rows > 0 ? (int)conflict_map[0].size() : 0;

    for (size_t i = 0; i < conflict_points.size(); ++i) {
        if (diamond_used[i] || solved_conflict_indices.count(i)) continue;
        std::cout << "[LOCAL ZONE] Building diamond bucket for conflict " << i << std::endl;
        std::vector<std::pair<int,int>> bucket_conflicts;
        bucket_conflicts.push_back(conflict_points[i]);
        // Collect conflict indices as we grow the bucket to ensure buckets always carry conflicts
        std::set<int> index_set;
        index_set.insert((int)i);
        // Track earliest timestep incrementally
        int bucket_earliest_t = conflict_meta[i].timestep; //set to timestep of first conflict
        int bucket_latest_t = conflict_meta[i].timestep;
        //set time window relvant for this conflict

        std::set<std::pair<int,int>> previous_shape;
        bool found_new_conflicts = true;
        while (found_new_conflicts) {
            found_new_conflicts = false;
            // Use conflict_map dimensions implicitly via map overloads
            auto current_shape = create_shape_from_conflicts(bucket_conflicts, offset, map);
            std::set<std::pair<int,int>> check_shape;

            auto new_positions = find_new_positions(current_shape, previous_shape, map);


            for (const auto& pos : new_positions) {
                auto [r, c] = pos;
                // Bounds guard to avoid OOB access
                if (r < 0 || r >= rows || c < 0 || c >= cols) {
                    std::cerr << "[LOCAL ZONE] ERROR: Found position (" << r << "," << c << ") outside of map bounds" << std::endl;
                    continue;
                }
                if (conflict_map[r][c].empty()) continue;
                for (int conflict_idx : conflict_map[r][c]) {
                    // Index validity guard
                    if (conflict_idx < 0 || conflict_idx >= (int)conflict_points.size()) {
                        std::cerr << "[LOCAL ZONE] ERROR: Found invalid conflict index " << conflict_idx << std::endl;
                        continue;
                    }
                    if (diamond_used[conflict_idx]) {
                        if (index_set.count(conflict_idx)==0) {
                                std::cerr << "[LOCAL ZONE] ERROR: Found already used conflict " << conflict_idx
                                            << " at position (" << r << "," << c
                                            << ") in new positions of diamond bucket." << std::endl;
                        }
                        continue;
                    }
                    if (!solved_conflict_indices.count(conflict_idx)) {
                        bucket_conflicts.push_back(conflict_points[conflict_idx]);
                        diamond_used[conflict_idx] = 1;
                        found_new_conflicts = true;
                        index_set.insert(conflict_idx);
                        if (conflict_idx >= 0 && conflict_idx < (int)conflict_meta.size()) {
                            bucket_earliest_t = std::min(bucket_earliest_t, conflict_meta[conflict_idx].timestep); //update earliest timestep of the bucket
                            bucket_latest_t = std::max(bucket_latest_t, conflict_meta[conflict_idx].timestep);                         
                        }
                    }
                }
            }

            previous_shape = std::move(current_shape);
        }

        auto diamond_positions = create_shape_from_conflicts(bucket_conflicts, offset, map);
        DiamondBucket diamond_bucket;
        diamond_bucket.positions = std::move(diamond_positions);
        
        diamond_bucket.indices.assign(index_set.begin(), index_set.end());
        diamond_bucket.earliest_t = bucket_earliest_t;
        diamond_bucket.latest_t = bucket_latest_t;

        diamond_buckets.push_back(std::move(diamond_bucket));
    }

    return diamond_buckets;
}


//check for earliest conflicts and build buckets around them
std::vector<DiamondBucket> build_diamond_buckets_for_earliest_conflicts(
    const std::vector<std::pair<int,int>>& conflict_points,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<std::vector<char>>& map,
    const std::vector<ConflictMeta>& conflict_meta,
    int offset) {
    
    //detect earliest conflicts
    std::set<ConflictMeta> earliest_conflicts;
    int earliest_t = INT_MAX;
    std::vector<int> original_earliest_conflict_indices;
    for (ConflictMeta conflict : conflict_meta) {
        if (conflict.timestep < earliest_t) {
            earliest_t = conflict.timestep;
            earliest_conflicts.clear();
            earliest_conflicts.insert(conflict);
            original_earliest_conflict_indices.clear();
            original_earliest_conflict_indices.push_back(conflict.idx);
        } else if (conflict.timestep == earliest_t) {
            earliest_conflicts.insert(conflict);
            original_earliest_conflict_indices.push_back(conflict.idx);
        }
    }
    //build metadata for earliest conflicts
    std::vector<std::pair<int,int>> earliest_conflict_positions;
    std::vector<ConflictMeta> earliest_conflict_meta; //correct indices (double presence of edge conflicts as they have 2 positions)
    for (ConflictMeta conflict : earliest_conflicts) {
        if (conflict.is_edge) {
            earliest_conflict_positions.push_back(conflict.pos1);
            earliest_conflict_positions.push_back(conflict.pos2);
            earliest_conflict_meta.push_back(conflict);
            earliest_conflict_meta.push_back(conflict);
        } else {
            earliest_conflict_positions.push_back(conflict.pos1);
            earliest_conflict_meta.push_back(conflict);
        }
    }
    if (!earliest_conflict_positions.empty()) {
        earliest_diamond_buckets = build_diamond_buckets(earliest_conflict_positions, conflict_map, map, earliest_conflict_meta, offset);   
    } else {
        std::cout << "[LOCAL ZONE] ERROR: No earliest conflicts found" << std::endl;
        return std::vector<DiamondBucket>();
    }
    //correct indices to original earliest conflict indices
    for (DiamondBucket bucket : earliest_diamond_buckets) {
        for (int idx : bucket.indices) {
            bucket.indices[idx] = original_earliest_conflict_indices[idx];
        }
    }
    return earliest_diamond_buckets;   
}

/**
 * @brief Create or expand an existing bucket's zone with an offset and absorb new conflicts.
 *
 * Computes the diamond shape with \p expanded_offset around the bucket's conflicts,
 * then iteratively includes additional conflicts discovered on the newly exposed frontier.
 * Returns the final expanded zone and the full set of conflict indices inside it.
 *
 * @param conflict_points          Conflict positions by index.
 * @param conflict_map             (r,c) -> list of conflict indices mapping.
 * @param map                      Map for bounds/walkability checks.
 * @param conflict_meta            Metadata for conflicts (for logging/consistency checks).
 * @param solved_conflict_indices  Conflicts to ignore (treated as already resolved).
 * @param original_bucket_indices  Indices that belong to the original bucket.
 * @param original_bucket_positions Positions comprising the original bucket zone.
 * @param expanded_offset          New diamond radius for expansion.
 * @return Pair {expanded_positions, expanded_conflict_indices}.
 *
 * @pre original indices/positions consistently describe the same bucket.
 * @post Returned indices include originals plus any newly touched conflicts.
 * @sideeffect Writes to stdout for diagnostics.
 * @complexity Depends on newly exposed frontier; typically near-linear in discovered area.
 */
std::pair<std::set<std::pair<int,int>>, std::vector<int>> expand_bucket_zone(
    const std::vector<std::pair<int,int>>& conflict_points,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<std::vector<char>>& map,
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<int>& original_bucket_indices,
    const std::set<std::pair<int,int>>& original_bucket_positions,
    int expanded_offset)
{
    //extract the conflicts from the original bucket
    std::vector<std::pair<int,int>> expanded_bucket_conflicts;
    std::vector<int> expanded_conflict_indices;
    for (int idx : original_bucket_indices) {
        if (idx >= 0 && idx < (int)conflict_points.size()) {
            expanded_bucket_conflicts.push_back(conflict_points[idx]);
            expanded_conflict_indices.push_back(idx);
        }
    }
    // Create expanded diamond shape with increased offset
    auto expanded_diamond_positions = create_shape_from_conflicts(expanded_bucket_conflicts, expanded_offset, map);
    // Find and merge newly touched conflicts in the expanded zone
    // Initialize previous_bucket_shape for iterative conflict discovery
    // Start with the original bucket shape so we catch conflicts from the initial expansion
    std::set<std::pair<int,int>> previous_bucket_shape(original_bucket_positions.begin(), original_bucket_positions.end());
    auto new_positions_in_expansion = find_new_positions(expanded_diamond_positions, previous_bucket_shape, map);

    std::vector<int> newly_touched_conflicts;
    for (const auto& pos : new_positions_in_expansion) {
        auto [r, c] = pos;
        if (conflict_map[r][c].empty()) continue;
        for (int conflict_idx : conflict_map[r][c]) {
            if (solved_conflict_indices.find(conflict_idx) != solved_conflict_indices.end()) continue; //check if the conflict is already solved
            bool was_in_original_bucket = false;
            for (int original_idx : original_bucket_indices) {
                if (original_idx == conflict_idx) { 
                    was_in_original_bucket = true; 
                    std::cout << "[LOCAL ZONE] ERROR: Conflict " << conflict_idx << " was already in the original bucket" << std::endl;
                    break; }
            }
            if (!was_in_original_bucket) {
                newly_touched_conflicts.push_back(conflict_idx);
                std::cout << "[LOCAL ZONE] Found newly touched conflict " << conflict_idx << " at position (" << r << "," << c << ")" << std::endl;
            }
        }
    }

    if (!newly_touched_conflicts.empty()) { //we have new conflicts to add to the expanded bucket, need to expand the bucket further
       
        for (int new_conflict_idx : newly_touched_conflicts) { //add the new conflicts to the expanded bucket
            if (new_conflict_idx >= 0 && new_conflict_idx < (int)conflict_points.size()) {
                expanded_bucket_conflicts.push_back(conflict_points[new_conflict_idx]);
                expanded_conflict_indices.push_back(new_conflict_idx);
            }
        }
        //set previous bucket shape to the expanded diamond positions
        previous_bucket_shape = std::set<std::pair<int,int>>(expanded_diamond_positions.begin(), expanded_diamond_positions.end());
        
        bool found_more_conflicts = true; //we need to check for more conflicts in the newly gained area
        while (found_more_conflicts) {
            found_more_conflicts = false;
            //create the new expanded diamond shape with the merged conflicts
            auto current_expanded_shape = create_shape_from_conflicts(expanded_bucket_conflicts, expanded_offset, map);
            auto new_positions_from_merged = find_new_positions(current_expanded_shape, previous_bucket_shape);
            for (const auto& pos : new_positions_from_merged) {
                auto [r, c] = pos;
                if (conflict_map[r][c].empty()) continue; //check if there are conflicts at the position
                for (int conflict_idx : conflict_map[r][c]) {
                    if (solved_conflict_indices.find(conflict_idx) != solved_conflict_indices.end()) continue; //check if the conflict is already solved
                    bool already_in_expanded_bucket = false;
                    for (int existing_idx : expanded_conflict_indices) {
                        if (existing_idx == conflict_idx) { 
                            already_in_expanded_bucket = true; 
                            std::cout << "[LOCAL ZONE] ERROR: Conflict " << conflict_idx << " was already in the expanded bucket" << std::endl;
                            break; 
                        }
                    }
                    if (!already_in_expanded_bucket) {
                        expanded_bucket_conflicts.push_back(conflict_points[conflict_idx]);
                        expanded_conflict_indices.push_back(conflict_idx);
                        newly_touched_conflicts.push_back(conflict_idx);
                        found_more_conflicts = true;
                    }
                }
            }
            previous_bucket_shape = std::set<std::pair<int,int>>(current_expanded_shape.begin(), current_expanded_shape.end());
        }
    }

    auto expanded_zone_positions_set = create_shape_from_conflicts(expanded_bucket_conflicts, expanded_offset, map);
    return {expanded_zone_positions_set, expanded_conflict_indices};
}

/**
 * @brief Extract collision records for a bucket's conflict indices.
 *
 * Iterates over \p bucket_indices (conflict indices) and converts their metadata into aggregated
 * vertex (agent1, agent2, pos1, timestep)  and edge (agent1, agent2, pos1, pos2, timestep) collision structures. 
 * Edge collisions are de-duplicated as they are represented by two conflict positions.
 * 
 *
 * @param bucket_indices  Conflict indices that belong to a bucket.
 * @param conflict_meta   Metadata array of all conflicts indexed by conflict index.
 * @return `CollisionExtractionResult` with counts and lists of collisions.
 *
 * @pre All indices in \p bucket_indices are valid for \p conflict_meta.
 * @post `vertex_count` and `edge_count` equal the sizes of their respective lists.
 * @sideeffect Writes summary lines to stdout.
 * @complexity O(|bucket_indices| log |bucket_indices|) due to set based de-dup.
 */
CollisionExtractionResult extract_collisions_from_bucket(
    const std::vector<int>& bucket_indices,
    const std::vector<ConflictMeta>& conflict_meta) {
    
    CollisionExtractionResult result;
    result.vertex_count = 0;
    result.edge_count = 0;
    
    // Use a set to avoid duplicate edge collisions
    std::set<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> edge_collisions_set;
    
    for (int idx : bucket_indices) {
        if (idx < 0 || idx >= (int)conflict_meta.size()) continue;
        
        const auto& meta = conflict_meta[idx];
        
        // Print conflict info
        std::cout << "[LOCAL ZONE] Conflict " << idx << " agents: " << meta.agent1 << " and " << meta.agent2 << std::endl;
        std::cout << "[LOCAL ZONE] Conflict " << idx << " timestep: " << meta.timestep << std::endl;
        std::cout << "[LOCAL ZONE] Conflict " << idx << " positions: " << meta.pos1.first << "," << meta.pos1.second;
        
        if (!meta.is_edge) {
            // Vertex collision
            std::cout << std::endl;
            result.vertex_collisions.emplace_back(meta.agent1, meta.agent2, meta.pos1, meta.timestep);
            result.vertex_count++;
        } else {
            // Edge collision
            std::cout << "[LOCAL ZONE] and " << meta.pos2.first << "," << meta.pos2.second << std::endl;
            
            // Check for duplicates
            auto edge_tuple = std::make_tuple(meta.agent1, meta.agent2, meta.pos1, meta.pos2, meta.timestep);
            if (edge_collisions_set.count(edge_tuple) == 0) {
                edge_collisions_set.insert(edge_tuple);
                result.edge_collisions.emplace_back(meta.agent1, meta.agent2, meta.pos1, meta.pos2, meta.timestep);
                result.edge_count++;
            }
        }
    }
    
    return result;
}

/**
 * @brief Validate that each conflict index is assigned to exactly one bucket.
 *
 * Logs errors if a conflict appears in multiple buckets or if some conflicts
 * are not assigned to any bucket.
 */
bool validate_buckets_cover_conflicts(
    const std::vector<std::pair<int,int>>& conflict_points,
    const std::vector<DiamondBucket>& diamond_buckets) {
    std::vector<char> used(conflict_points.size(), 0);
    int num_used = 0;
    for (const auto& bucket : diamond_buckets) {
        for (int idx : bucket.indices) {
            if (idx < 0 || idx >= (int)conflict_points.size()) {
                std::cout << "[LNS] ERROR: Conflict index out of range: " << idx << std::endl;
                continue;
            }
            if (used[idx]) {
                std::cout << "[LNS] ERROR: Conflict " << idx << " was assigned to multiple diamond buckets" << std::endl;
                return false;
            }
            used[idx] = 1;
            num_used++;
        }
    }
    if (num_used != (int)conflict_points.size()) {
        std::cout << "[LNS] ERROR: " << (int)conflict_points.size() - num_used
                  << " conflicts were not assigned to any diamond bucket" << std::endl;
        return false;
    }
    return true;
}

DiamondBucket select_most_relevant_bucket(const std::vector<DiamondBucket>& diamond_buckets) {
    if (diamond_buckets.empty()) return DiamondBucket();
    int best_bucket_idx = -1;
    int best_conflict_pos_count = -1;
    int best_shape_size = -1;
    for (size_t bi = 0; bi < diamond_buckets.size(); ++bi) {
        const auto& b = diamond_buckets[bi];
        int conflict_pos_count = static_cast<int>(b.indices.size());
        int shape_size = static_cast<int>(b.positions.size());
        if (conflict_pos_count > best_conflict_pos_count ||
            (conflict_pos_count == best_conflict_pos_count && shape_size > best_shape_size)) {
            best_bucket_idx = static_cast<int>(bi);
            best_conflict_pos_count = conflict_pos_count;
            best_shape_size = shape_size;
        }
    }
    return diamond_buckets[best_bucket_idx];
}