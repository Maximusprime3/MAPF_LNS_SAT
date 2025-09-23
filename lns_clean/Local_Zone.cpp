#include "Local_Zone.h"
#include "Current_Solution.h"

#include <cmath>
#include <algorithm>  // for std::set_difference
#include <iterator>   // for std::inserter

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


// Overload: restrict using the map itself
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


// Overload: restrict using the map itself
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


// Helper: form diamond shaped buckets around conflicts.  Each conflict belongs
// to exactly one bucket.  The algorithm incrementally grows a diamond around a
// seed conflict, merging nearby conflicts that fall into the diamond.
static std::vector<DiamondBucket> build_diamond_buckets(
    const std::vector<std::pair<int,int>>& conflict_points,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<std::vector<char>>& map,
    const std::set<int>& solved_conflict_indices,
    const std::vector<ConflictMeta>& conflict_meta,
    int offset) {

    std::vector<DiamondBucket> diamond_buckets;
    std::vector<char> diamond_used(conflict_points.size(), 0);
    // Dimensions for safe conflict_map access
    const int rows = (int)conflict_map.size();
    const int cols = rows > 0 ? (int)conflict_map[0].size() : 0;

    for (size_t i = 0; i < conflict_points.size(); ++i) {
        if (diamond_used[i] || solved_conflict_indices.count(i)) continue;
        std::cout << "[LNS] Building diamond bucket for conflict " << i << std::endl;
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
                    std::cerr << "[ERROR] Found position (" << r << "," << c << ") outside of map bounds" << std::endl;
                    continue;
                }
                if (conflict_map[r][c].empty()) continue;
                for (int conflict_idx : conflict_map[r][c]) {
                    // Index validity guard
                    if (conflict_idx < 0 || conflict_idx >= (int)conflict_points.size()) {
                        std::cerr << "[ERROR] Found invalid conflict index " << conflict_idx << std::endl;
                        continue;
                    }
                    if (diamond_used[conflict_idx]) {
                        if (index_set.count(conflict_idx)==0) {
                                std::cerr << "[ERROR] Found already used conflict " << conflict_idx
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


// Helper: expand a bucket by increasing offset and iteratively adding newly touched conflicts
// Returns the expanded positions set and the list of conflict indices contained in the expanded zone
static std::pair<std::set<std::pair<int,int>>, std::vector<int>> expand_bucket_zone(
    const std::vector<std::pair<int,int>>& conflict_points,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<std::vector<char>>& map,
    const std::vector<ConflictMeta>& conflict_meta,
    const std::set<int>& solved_conflict_indices,
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
                    std::cout << "[LNS] ERROR: Conflict " << conflict_idx << " was already in the original bucket" << std::endl;
                    break; }
            }
            if (!was_in_original_bucket) {
                newly_touched_conflicts.push_back(conflict_idx);
                std::cout << "[LNS] Found newly touched conflict " << conflict_idx << " at position (" << r << "," << c << ")" << std::endl;
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
                            std::cout << "[LNS] ERROR: Conflict " << conflict_idx << " was already in the expanded bucket" << std::endl;
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