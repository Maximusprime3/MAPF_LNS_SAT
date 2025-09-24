#include "Local_Zone.h" //expand_bucket_zone
#include "Current_Solution.h"
#include "DiamondBucket.h"
#include "ConflictMeta.h"
#include "LazySolveResult.h"


// Helper function to find agents present in a zone within a time window
std::set<int> get_agents_in_zone_within_time_window(
    const CurrentSolution& current_solution,
    const std::set<std::pair<int,int>>& zone_positions_set,
    int start_t,
    int end_t) {
    
    std::set<int> agents_in_window;
    for (const auto& rc : zone_positions_set) {
        int r = rc.first, c = rc.second;
        for (int t = start_t; t <= end_t; ++t) {
            auto here = current_solution.get_agents_at_position_time(r, c, t);
            agents_in_window.insert(here.begin(), here.end());
        }
    }
    return agents_in_window;
}



LocalZoneResult solve_local_zone(
    const std::vector<std::vector<char>>& map, 
    const DiamondBucket& best_bucket, 
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<std::vector<std::vector<int>>>& conflict_map, 
    CurrentSolution& current_solution, 
    int offset,
    int current_max_timesteps) {
        
    LocalZoneResult local_zone_result;
    local_zone_result.solution_found = false;
    local_zone_result.local_paths = std::unordered_map<int, std::vector<std::pair<int,int>>>();
    local_zone_result.local_entry_exit_time = std::vector<int>();
    

    //get number of all walkable positions in the map
    int all_walkable_positions = 0;
    for (int i = 0; i < map.size(); i++) {
        for (int j = 0; j < map[0].size(); j++) {
            if (map[i][j] == '.') {
                all_walkable_positions++;
            }
        }
    }
    std::set<std::pair<int,int>> local_zone_positions = best_bucket.positions;
    std::vector<int> local_zone_conflict_indices = best_bucket.indices;

    int expansion_factor = 0;
    //loop until solution found or the local zone reached the size of the map and still no solution found
    while (local_zone_result.solution_found == false && local_zone_positions.size() < all_walkable_positions) {
        //current local zone size and % or all walkable positions
        std::cout << "[Solve_local_zone] Current local zone size: " << local_zone_positions.size() << " (" 
        << (double)local_zone_positions.size() / all_walkable_positions * 100 << "% of all walkable positions)" << std::endl;
        
        //Step 1: create local problem
        auto local_masked_map = mask_map_outside_shape(map, local_zone_positions);//all positions outside the local zone are not walkable
        //set start and end time for the local zone
        int start_t = std::max(0, local_zone_conflict_indices[0] - offset);
        int end_t = std::min(current_max_timesteps, local_zone_conflict_indices[local_zone_conflict_indices.size() - 1] + offset);
        std::cout << "[Solve_local_zone] Local zone time window: [" << start_t << ", " << end_t << "]" << std::endl;
        //zone positions
        std::set<std::pair<int,int>> zone_positions_set(local_zone_positions.begin(), local_zone_positions.end());
        
        //Step 2: solve the local problem
        auto local_zone_result = lazy_solve_with_waiting_time(
            current_solution,
            map,
            local_masked_map,
            zone_positions_set,
            conflict_meta,
            local_zone_conflict_indices,
            conflict_map,
            start_t, end_t,
            offset);
        
        //if solution found, update the current solution
        if (local_zone_result.solution_found) {
            std::cout << "[Solve_local_zone] Successfully solved local zone" << std::endl;
            //integrate local zone result into current solution
            current_solution.update_with_local_paths(local_zone_result.local_paths, local_zone_result.local_entry_exit_time);
            break;
        }

        //if no solution found
        //Step 3: Expand the local zone 
        // Expansion attempts: increase bucket offset and try again
        std::cout << "[Solve_local_zone] Zone with expansion factor " << expansion_factor << "failed" << std::endl;
        expansion_factor++;
        int expanded_offset = offset + expansion_factor;
        std::cout << "[Solve_local_zone] Expanding zone with offset " << expanded_offset << " (original: " << offset << ")" << std::endl;
        // Recreate the bucket conflicts from the original bucket indices
        // Use helper to expand zone and gather expanded conflict indices
        //todo:no more conflict points? all conflict meta
        auto [expanded_zone_positions_set, expanded_conflict_indices, new_time_window] = expand_bucket_zone(
            conflict_points,
            conflict_map,
            map,
            conflict_meta,
            local_zone_conflict_indices,
            local_zone_positions,
            expanded_offset);
        
        local_zone_positions = expanded_zone_positions_set;
        local_zone_conflict_indices = expanded_conflict_indices; //todo: no more conflict indices? all conflict meta
        start_t = new_time_window.first;
        end_t = new_time_window.second;

        std::cout << "[Solve_local_zone] Final expanded zone contains " << expanded_zone_positions_set.size() 
                  << " positions with " << expanded_conflict_indices.size() << " conflicts << std::endl;" << std::endl; 
        std::cout << "[Solve_local_zone] New time window: [" << start_t << ", " << end_t << "]" << std::endl;
        

    }

    


    
    return local_zone_result;
}