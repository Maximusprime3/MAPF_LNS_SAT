#include "Create_Local_Problem.h"

#include "../mdd/MDDConstructor.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>


// Forward declaration provided by Solve_Local_Zone.cpp.
std::set<int> get_agents_in_zone_within_time_window(
    const CurrentSolution& current_solution,
    const std::set<std::pair<int,int>>& zone_positions_set,
    int start_t,
    int end_t);


namespace {

    struct AgentPathSegment {
        std::vector<std::pair<int,int>> contiguous_intervals; //entry and exit times of the segments
        int entry_t = -1;
        int exit_t = -1;
        bool agent_in_zone = false;
    };

    struct AgentProcessingResult {
        std::vector<std::vector<std::pair<int,int>>> zone_paths;
        std::vector<int> entry_t;
        std::vector<int> exit_t;
        std::set<std::pair<int,int>> expanded_zone_positions;
        std::vector<int> additional_agents;
        std::vector<ConflictMeta> additional_conflicts;
    };

    AgentPathSegment extract_agent_path_segment(
        int agent_id,
        const std::vector<std::pair<int,int>>& path,
        const std::set<std::pair<int,int>>& zone_positions_set,
        int start_t,
        int end_t) {
        
        AgentPathSegment segment;
        
        
        const int window_span = end_t - start_t + 1;
        if (window_span > 0) {    
            segment.contiguous_intervals.reserve(window_span);
        }
    
        bool agent_in_zone = false;
        // Getting path segments in the zone
        for (int t = start_t; t <= end_t && t < static_cast<int>(path.size()); ++t) {
            const auto& pos = path[t]; // walking along the agent's path
            bool pos_in_zone = zone_positions_set.count(pos) > 0;
            if (pos_in_zone) { // agent is in the zone
                if (!agent_in_zone) { // agent is entering the zone
                    segment.agent_in_zone = true;
                    agent_in_zone = true;
                    if (segment.entry_t == -1) {
                        segment.entry_t = t;
                    }
                    //std::cout << "[Create_Local_problem] Agent " << agent_id << " entered the zone at timestep " << t << std::endl;
                    //print path
                    //std::cout << "[Create_Local_problem] Path (size: " << path.size() << "): ";
                    //for (const auto& pos : path) {
                        //std::cout << "(" << pos.first << ", " << pos.second << ") ";
                    //}
                    //std::cout << std::endl;
                    segment.contiguous_intervals.emplace_back(t, t); // start new interval
                } else { // agent is already in the zone
                    segment.contiguous_intervals.back().second = t; // last known position in the zone
                }
                segment.exit_t = t; 
            } else if (agent_in_zone) { // agent is leaving the zone
                segment.contiguous_intervals.back().second = segment.exit_t; // end of interval in the zone
                //std::cout << "[Create_Local_problem] Agent " << agent_id << " is not in the zone at timestep " << t << std::endl;
                //std::cout << "The exit time is " << segment.exit_t << std::endl;
                agent_in_zone = false;
            }       
        }
        
        if (!segment.contiguous_intervals.empty() && segment.agent_in_zone) {
            segment.contiguous_intervals.back().second = segment.exit_t; 
            std::cout << "[Create_Local_problem] Agent " << agent_id << " exited the zone at exit time " << segment.exit_t << std::endl;
        }
        
        return segment;
    }


    // helper function to process agent paths and handle zone expansion
    //takes global path and extracts the segment of the path that is inside the zone
    AgentProcessingResult process_agent_in_zone(
        int agent_id,
        const std::vector<std::pair<int,int>>& path,
        const std::set<std::pair<int,int>>& zone_positions_set,
        const std::vector<std::vector<std::vector<int>>>& conflict_map,
        const std::vector<ConflictMeta>& conflict_meta,
        int start_t,
        int end_t,
        int offset,
        const std::vector<std::vector<char>>& grid) {

        (void)conflict_map;
        (void)conflict_meta;
        (void)offset;
        (void)grid;
        
        AgentProcessingResult result;
        
        auto segment_info = extract_agent_path_segment(agent_id, path, zone_positions_set, start_t, end_t);
        
        if (!segment_info.contiguous_intervals.empty()) {
            
            if (segment_info.contiguous_intervals.size() > 1) {
                std::cout << "[Create_Local_problem] Agent " << agent_id << " returned to the zone "
                        << (segment_info.contiguous_intervals.size() - 1) << " times within time window" << std::endl;
            }
            //create seperate paths for the returning agent
            for (const auto& interval : segment_info.contiguous_intervals) {
                if (interval.first < 0 || interval.second >= static_cast<int>(path.size())) {
                    std::cout << "[Create_Local_problem] ERROR: Invalid interval for agent " << agent_id << ": [" << interval.first << ", " << interval.second << "]" << std::endl;
                    continue;
                }
                result.zone_paths.push_back(std::vector<std::pair<int,int>>(
                    path.begin() + interval.first,
                    path.begin() + interval.second + 1
                ));
                result.entry_t.push_back(interval.first);
                result.exit_t.push_back(interval.second);
            }
            

        }else{
            std::cout << "[Create_Local_problem] ERROR: Agent " << agent_id << " has no path in the zone" << std::endl;
        }
        
        return result;
    }

    //extend the mdd with a waiting tail to the goal position
    //if want to create only waiting mdd, create mdd with only goal position, then use this function
    bool extend_waiting_suffix_in_mdd(std::shared_ptr<MDD> mdd,
        int old_exit_t,
        int new_exit_t,
        const std::pair<int,int>& goal_pos) {
        
        if (!mdd || mdd->levels.empty() || old_exit_t > new_exit_t) {
            std::cout << "[Create_Local_Problem] ERROR: No MDD to extend" << std::endl;
            return false;
        }
        if (old_exit_t == new_exit_t) {
            std::cout << "[Create_Local_Problem] Old exit time is equal to new exit time, no need to extend" << std::endl;
            return true;
        }
        
        auto parent_level_it = mdd->levels.find(old_exit_t);
        if (parent_level_it == mdd->levels.end()) {
            std::cout << "[Create_Local_Problem] ERROR: No MDD level found at old exit time "
                      << old_exit_t << std::endl;
            return false;
        }
        auto& parent_level = parent_level_it->second;
        if (parent_level.empty()) {
            std::cout << "[Create_Local_Problem] ERROR: Parent level is empty" << std::endl;
            return false;
        }
        
        bool has_goal_parent = false;
        for (const auto& node : parent_level) {
            if (node && node->position == goal_pos) {
                has_goal_parent = true;
                break;
            }
        }
        if (!has_goal_parent) {
            std::cout << "[Create_Local_Problem] ERROR: No goal parent found" << std::endl;
            return false;
        }
        
        for (int t = old_exit_t + 1; t <= new_exit_t; ++t) {
            auto waiting_node = std::make_shared<MDDNode>(goal_pos, t);

            auto parent_level_it = mdd->levels.find(t-1);
            if (parent_level_it == mdd->levels.end() || parent_level_it->second.empty()) {
                std::cout << "[Create_Local_Problem] ERROR: No MDD level found at time " << t-1 << std::endl;
                return false;
            }

            auto& parent_level = parent_level_it->second;
            auto parent_node = parent_level.front();
            //there should be only one node at this level but safety check
            if (parent_level.size() != 1) {
                std::cout << "[Create_Local_Problem] ERROR: More than one node at final mdd level " << t-1 << std::endl;
                for (const auto& node : parent_level) {
                    std::cout << "[Create_Local_Problem] Node: " << node->position.first << ", " << node->position.second << " at time " << node->time_step << std::endl;
                }
                return false;
            } else if (parent_node->position != goal_pos) { //parent node should be the goal position
                std::cout << "[Create_Local_Problem] ERROR: Parent node is not the goal position" << std::endl;
                std::cout << "[Create_Local_Problem] Parent node: " << parent_node->position.first << ", " << parent_node->position.second << " at time " << parent_node->time_step << std::endl;
                return false;
            }
            //current mdd level should be empty
            auto& current_level = mdd->levels[t];
            if (!current_level.empty()) {
                std::cout << "[Create_Local_Problem] ERROR: MDD level " << t << " is not empty" << std::endl;
                for (const auto& node : current_level) {
                    std::cout << "[Create_Local_Problem] Node: " << node->position.first << ", " << node->position.second << " at time " << node->time_step << std::endl;
                }
                return false;
            }
            parent_node->add_child(waiting_node);
            mdd->add_node(waiting_node);
        }
        return true;
    }

}




std::shared_ptr<MDD> build_segment_mdd_with_optional_wait_tail(
    const std::vector<std::vector<char>>& masked_map,
    const std::vector<std::pair<int,int>>& segment_path,
    const std::pair<int,int>& global_goal_pos,
    int segment_entry_t,
    int segment_exit_t,
    int window_start_t,
    int window_end_t,
    int agent_id,
    int forced_pre_tail_idx = -1) {

    if (segment_path.empty()) {
        std::cout << "[Create_Local_Problem] ERROR: Empty segment path for agent " << agent_id << std::endl;
        return nullptr;
    }
    const std::pair<int,int>& start_pos = segment_path.front();
    const std::pair<int,int>& goal_pos = segment_path.back();

    const auto it = std::find(segment_path.begin(), segment_path.end(), global_goal_pos);
    if (forced_pre_tail_idx != -1) {
        it = segment_path.begin() + forced_pre_tail_idx; //this is used to extend mdd body before the tail begins. to use waiting time
    }
    if (it != segment_path.end()) {
        const int idx = static_cast<int>(std::distance(segment_path.begin(), it));
        MDDConstructor constructor(masked_map, start_pos, global_goal_pos, std::max(0, idx));
        auto mdd = constructor.construct_mdd();

        int start_of_waiting_suffix = segment_entry_t + idx;
        align_mdd_to_time_window(mdd, segment_entry_t, start_of_waiting_suffix, window_start_t, window_end_t);
        // verify suffix waits at goal
        for (int i = idx + 1; i < static_cast<int>(segment_path.size()); ++i) {
            if (segment_path[i] != global_goal_pos) {
                std::cout << "[Create_Local_Problem] ERROR: Agent " << agent_id << " deviates after reaching global goal" << std::endl;
                break;
            }
        }
        
        bool ok = extend_waiting_suffix_in_mdd(mdd, start_of_waiting_suffix, segment_exit_t, global_goal_pos);
        if (!ok) {
            std::cout << "[Create_Local_Problem] ERROR: Failed to extend waiting tail for agent " << agent_id << std::endl;
            return nullptr;
        }
        return mdd;
    }

    // normal MDD to local segment goal
    const int segment_length = std::max(0, segment_exit_t - segment_entry_t + 1);
    MDDConstructor constructor(masked_map, start_pos, goal_pos, std::max(0, segment_length - 1));
    auto mdd = constructor.construct_mdd();
    align_mdd_to_time_window(mdd, segment_entry_t, segment_exit_t, window_start_t, window_end_t);
    return mdd;
}

std::shared_ptr<MDD> build_segment_mdd(
    Current_Solution current_solution,
    LocalSegment segment,
    const std::vector<std::vector<char>>& masked_map,
    int window_start_t,
    int window_end_t){
    
    const std::vector<std::pair<int,int>>& segment_path = segment.path;
    int segment_entry_t = segment.entry_t;
    int segment_exit_t = segment.exti_t;
    int agent_id = segment.original_id;
    const std::pair<int,int>& global_goal_pos = ccurrent_solution.goals[agent_id];

    if (segment_path.empty()) {
        std::cout << "[Create_Local_Problem] ERROR: Empty segment path for agent " << agent_id << std::endl;
        return nullptr;
    }
    const std::pair<int,int>& start_pos = segment_path.front();
    const std::pair<int,int>& goal_pos = segment_path.back();

    const int segment_length = std::max(0, segment_exit_t - segment_entry_t + 1);

    const auto it = std::find(segment_path.begin(), segment_path.end(), global_goal_pos);
    //if the goal is on the segment path
    if (it != segment_path.end()) {
        //where in the path is it
        const int idx = static_cast<int>(std::distance(segment_path.begin(), it));
        //how much waiting time will be used
        const int used_waiting_time = segment_exit_t - segment_entry_t - idx;
        //use that much waiting time
        current_solution.use_waiting_time(agent_id, used_waiting_time);
    }
    // normal MDD to local segment goal
    
    MDDConstructor constructor(masked_map, start_pos, goal_pos, std::max(0, segment_length - 1));
    auto mdd = constructor.construct_mdd();
    align_mdd_to_time_window(mdd, segment_entry_t, segment_exit_t, window_start_t, window_end_t);
    return mdd;
    
}


void align_mdd_to_time_window(std::shared_ptr<MDD> mdd,
    int entry_t, int exit_t, //agent's entry and exit times
    int start_t, int end_t) { //time window of problem zone start and end

    //std::cout << "[Create_Local_Problem] Starting to align MDD to the time window: [" << start_t << ", " << end_t << "]" << std::endl;
    if (!mdd) { // do we have an mdd?
        std::cout << "[Create_Local_Problem] ERROR: No MDD to align" << std::endl;
        return;
    }

    //std::cout << "[Create_Local_Problem] MDD levels: " << mdd->levels.size() << std::endl;
    //is it empty?
    if (mdd->levels.empty()) {
        std::cout << "[Create_Local_Problem] ERROR: MDD is empty" << std::endl;
        return;
    }

    if (end_t < start_t) { // does start and end make sense?
        std::cout << "[Create_Local_Problem] ERROR: Invalid time window for MDD alignment (end_t < start_t)." << std::endl;
        mdd->levels.clear();
        return;
    }
    if (entry_t > exit_t) {
        std::cout << "[Create_Local_Problem] ERROR: Invalid entry and exit times for MDD alignment (entry_t > exit_t)." << std::endl;
        mdd->levels.clear();
        return;
    }

    int zone_mdd_length = end_t - start_t + 1;
    // Calculate relative timesteps within the time window
    int relative_entry = entry_t - start_t;  // 0-based within the time window
    int relative_exit = exit_t - start_t;    // 0-based within the time window

    relative_entry = std::max(0, std::min(relative_entry, zone_mdd_length - 1));
    relative_exit = std::max(relative_entry, std::min(relative_exit, zone_mdd_length - 1)); 

    auto original_levels = mdd->levels;
    std::map<int, std::vector<std::shared_ptr<MDDNode>>> aligned_levels;

    //check if already aligned to the time window 
    //if first MDD level matches entry
    bool mdd_start_aligned = false;
    if (original_levels.begin()->first == relative_entry + start_t) {
        //std::cout << "[Create_Local_Problem] MDD start " << original_levels.begin()->first << " already at relative entry: " << relative_entry + start_t << std::endl;
        mdd_start_aligned = true;
    }

    //if last MDD level matches exit
    bool mdd_end_aligned = false;
    // get last MDD level 
    if (original_levels.rbegin()->first == relative_exit + start_t) {
        //std::cout << "[Create_Local_Problem] MDD end " << original_levels.rbegin()->first << " already aligned to the time window: " << relative_exit + start_t << std::endl;
        mdd_end_aligned = true;
    }

    //std::cout << "checked if MDD start and end are aligned" << std::endl;
    if (mdd_start_aligned && mdd_end_aligned) {
        std::cout << "[Create_Local_Problem] MDD already aligned to the time window" << std::endl;
        return;
    }
    // Shift the agent's MDD levels to the correct position. Some MDDs already use
    // absolute timesteps (their first level matches the agent's entry time), while
    // others are stored relative to 0.  We therefore compute the offset required to
    // place the first level at the agent's entry time and then clip anything outside
    // of the zone's window.
    int first_level = original_levels.begin()->first;
    int offset = entry_t - first_level;
    int max_allowed_level = std::min(exit_t, start_t + zone_mdd_length - 1);
    if(exit_t > start_t + zone_mdd_length - 1) {
        std::cout << "[Create_Local_Problem] ERROR: Exit time " << exit_t << " exceeds zone MDD length " << start_t + zone_mdd_length - 1 << std::endl;
    }

    for (const auto& [level, nodes] : original_levels) {
        int new_level = level + offset;

        if (new_level < start_t) {
            std::cout << "[Create_Local_Problem] ERROR: MDD level " << new_level
                      << " is less than start time " << start_t << "; truncating." << std::endl;
            continue;
        }

        if (new_level > max_allowed_level) {
            std::cout << "[Create_Local_Problem] ERROR: MDD level " << new_level
                      << " exceeds time window length " << max_allowed_level
                      << "; truncating." << std::endl;
            continue;
        }

        auto& target_nodes = aligned_levels[new_level]; //get the target nodes
        target_nodes = nodes;
        for (auto& node : target_nodes) {
            node->time_step = new_level; //MDD is now in absolute time scale similar to collision metadata and update logic 
        }
    }

    //std::cout << "[Create_Local_Problem] Aligned MDD to the time window" << std::endl;
    mdd->levels = std::move(aligned_levels);
}



LocalZoneState build_local_problem_for_zone(
    const CurrentSolution& current_solution,
    const std::set<std::pair<int,int>>& zone_positions_set,
    const std::vector<std::vector<char>>& masked_map,
    const std::vector<std::vector<char>>& grid,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<ConflictMeta>& conflict_meta,
    int offset,
    int start_t,
    int end_t,
    const std::unordered_map<int, std::vector<int>>& agent_to_pseudo_agent_id) {

    LocalZoneState state;
    state.zone_start_t = start_t;
    state.zone_end_t = end_t;
    state.original_to_pseudo_ids = agent_to_pseudo_agent_id;
    state.next_pseudo_id = static_cast<int>(agent_to_pseudo_agent_id.size());

    const int real_agent_count = static_cast<int>(current_solution.starts.size());
    int max_real_agent_id = real_agent_count > 0 ? real_agent_count - 1 : -1;
    int max_existing_agent_id = -1;
    for (const auto& [agent_id, _] : current_solution.agent_paths) {
        max_existing_agent_id = std::max(max_existing_agent_id, agent_id);
        if (agent_id < real_agent_count) {
            max_real_agent_id = std::max(max_real_agent_id, agent_id);
        }
    }
    state.next_pseudo_id = std::max(max_existing_agent_id + 1, max_real_agent_id + 1);
    for (const auto& [agent_id, pseudo_ids] : agent_to_pseudo_agent_id) {
        for (int pseudo_id : pseudo_ids) {
            if (pseudo_id <= max_real_agent_id) {
                std::cout << "[Create_Local_Problem] ERROR: Pseudo agent ID " << pseudo_id
                          << " overlaps with real agent range (max real ID " << max_real_agent_id
                          << ")" << std::endl;
            }
            state.next_pseudo_id = std::max(state.next_pseudo_id, pseudo_id + 1);
        }
    }
    // Find agents present in the zone within time window
    auto agents_in_window = get_agents_in_zone_within_time_window(current_solution, zone_positions_set, start_t, end_t);
    //check how many pseudo agents we have already
    //int number_of_pseudo_agents = agent_to_pseudo_agent_id.size();
    //const int global_number_of_agents = current_solution.agent_paths.size();
    // Extract segments and compute entry/exit
    for (int agent_id : agents_in_window) {
        const auto& path = current_solution.agent_paths.at(agent_id);
        auto agent_result = process_agent_in_zone(agent_id, path, zone_positions_set, conflict_map, conflict_meta, start_t, end_t, offset, grid);
        if (agent_result.zone_paths.empty()) {
            std::cout << "[Create_Local_problem] ERROR: Agent " << agent_id << " has no paths in the zone" << std::endl;
        }

        auto& pseudo_list = state.original_to_pseudo_ids[agent_id];
        size_t pseudo_cursor = 0;

        //go through all agents segments
        for ( size_t seg_idx = 0; seg_idx < agent_result.zone_paths.size(); ++seg_idx) {
            auto segment_id = agent_id;
            //expands the list of pseudo agents for the original agent if needed
            if (seg_idx > 0) {
                if (pseudo_cursor < pseudo_list.size()) {
                    segment_id = pseudo_list[pseudo_cursor++];
                }else{
                    if (state.next_pseudo_id <= max_real_agent_id) {
                        std::cerr << "[Create_Local_Problem] WARNING: Assigning pseudo ID "
                                  << state.next_pseudo_id
                                  << " that overlaps with real agent range (max real ID "
                                  << max_real_agent_id << ")" << std::endl;
                    }
                    segment_id = state.next_pseudo_id++;
                    pseudo_list.push_back(segment_id);
                    ++pseudo_cursor;
                }
            }

            LocalSegment segment;
            segment.segment_id = segment_id;
            segment.original_id = agent_id;
            segment.entry_t = agent_result.entry_t[seg_idx];
            segment.exit_t = agent_result.exit_t[seg_idx];
            segment.original_entry_t = agent_result.entry_t[seg_idx];
            segment.original_exit_t = agent_result.exit_t[seg_idx];
            segment.path = std::move(agent_result.zone_paths[seg_idx]);

            const auto& segment_path = segment.path;
            if (!segment_path.empty()) {
                int segment_length = segment.exit_t - segment.entry_t + 1;
                if (segment_length <= 0) {
                    segment_length = static_cast<int>(segment_path.size());
                }
                // build MDD, optionally with waiting tail if global goal is in the segment
                const auto& global_goal_pos = current_solution.goals[agent_id];
                segment.mdd = build_segment_mdd_with_optional_wait_tail(
                    masked_map,
                    segment_path,
                    global_goal_pos,
                    segment.entry_t,
                    segment.exit_t,
                    start_t,
                    end_t,
                    agent_id);
                if (!segment.mdd) {
                    std::cout << "[Create_Local_Problem] ERROR: Failed to build MDD for agent " << agent_id << std::endl;
                    continue;
                }
            }

            size_t new_index = state.segments.size();
            state.segment_index_by_id[segment.segment_id] = new_index;
            state.original_to_segments[segment.original_id].push_back(new_index);
            state.zone_end_t = std::max(state.zone_end_t, segment.exit_t);
            state.segments.push_back(std::move(segment));
        }

        //ensure original_to_segments order is sorted by entry_t
        auto& segment_indices = state.original_to_segments[agent_id];
        std::sort(segment_indices.begin(), segment_indices.end(), [&](size_t a, size_t b) {
            return state.segments[a].entry_t < state.segments[b].entry_t;
        });
        
    }

    return state;
}


std::unordered_map<int, std::shared_ptr<MDD>> build_segment_mdd_map(const LocalZoneState& state) {
    std::unordered_map<int, std::shared_ptr<MDD>> result;
    for (const auto& segment : state.segments) {
        result[segment.segment_id] = segment.mdd;
    }
    return result;
}

std::unordered_map<int, std::pair<int,int>> build_segment_entry_exit_time_map(const LocalZoneState& state) {
    std::unordered_map<int, std::pair<int,int>> result;
    for (const auto& segment : state.segments) {
        result[segment.segment_id] = {segment.entry_t, segment.exit_t};
    }
    return result;
}

std::unordered_map<int, std::vector<std::pair<int,int>>> build_segment_path_map(const LocalZoneState& state) {
    std::unordered_map<int, std::vector<std::pair<int,int>>> result;
    for (const auto& segment : state.segments) {
        result[segment.segment_id] = segment.path;
    }
    return result;
}



// When the local zone window is lengthened we need to rescan the agents that now
// fall inside the extended portion. This helper grows existing segments,
// creates pseudo segments for new/returning agents, rebuilds the required MDDs
// and updates the cached collision information so the next SAT attempt has a
// consistent view of the zone.
void refresh_zone_after_extension(
    LocalZoneState& state,
    CurrentSolution& current_solution,
    const std::set<std::pair<int,int>>& local_zone_positions,
    int previous_zone_end_t,
    const std::vector<std::vector<char>>& masked_map,
    const std::vector<std::vector<char>>& map,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<ConflictMeta>& conflict_meta,
    int offset) {

    if (state.zone_end_t <= previous_zone_end_t) {
        return;
    }
    //the window we need to check for new agents starts after the previous zone end time
    int new_window_start = previous_zone_end_t + 1;

    //get the agents that are in the new window
    auto newly_relevant_agents = current_solution.get_agents_in_zone(
        local_zone_positions,
        new_window_start,
        state.zone_end_t);

    if (newly_relevant_agents.empty()) {
        return;
    }

    std::set<int> agents_to_resort;

    //returns value between min and max
    auto clamp_time = [](int value, int min_value, int max_value) {
        return std::max(min_value, std::min(value, max_value));
    };

    //iterate through all agents that are in the new window
    for (int agent_id : newly_relevant_agents) {
        //get that agents global path
        auto path_it = current_solution.agent_paths.find(agent_id);
        if (path_it == current_solution.agent_paths.end()) {
            std::cout << "[Create_Local_Problem] WARNING: Missing global path for agent " << agent_id << std::endl;
            continue;
        }
        const auto& global_path = path_it->second;
        if (global_path.empty()) {
            std::cout << "[Create_Local_Problem] WARNING: Empty global path for agent "
                      << agent_id << std::endl;
            continue;
        }
        //verify waiting time
        int waiting_time = current_solution.get_waiting_time(agent_id);
        if (global_path.back() != current_solution.goals[agent_id]) {
            std::cout << "[Create_Local_Problem] ERROR: Agent " << agent_id << " does not end at the goal position" << std::endl;
            std::cout << "[Create_Local_Problem] Waiting time: " << waiting_time << std::endl;
            std::cout << "[Create_Local_Problem] Path (size: " << global_path.size() << "): ";
            for (const auto& pos : global_path) {
                std::cout << "(" << pos.first << ", " << pos.second << ") ";
            }
            std::cout << std::endl;
        }
        if (global_path[global_path.size() - waiting_time - 1] != current_solution.goals[agent_id]) {
            std::cout << "[Create_Local_Problem] ERROR: Agent " << agent_id << " does not end at the goal position with waiting time" << std::endl;

        }

        //get the max time of the path
        int path_length = static_cast<int>(global_path.size()) - 1; //should be makespan
        //check if its makespan
        if (path_length > current_solution.max_timestep) {
            std::cout << "[Create_Local_Problem] ERROR: Agent " << agent_id << " has path length " << path_length << " instead of makespan " << current_solution.max_timestep << std::endl;
        } 

        //clamp the start and end of the scan, cannot be negative or greater than the path length
        int scan_start = clamp_time(new_window_start, 0, path_length);
        int scan_end = clamp_time(state.zone_end_t, 0, path_length);
        if (scan_start > scan_end) {
            continue;
        }
        //get the agents segments in the extended zone window
        AgentProcessingResult segment_info = process_agent_in_zone(
            agent_id, 
            global_path, 
            local_zone_positions, 
            conflict_map, 
            conflict_meta, 
            scan_start, 
            scan_end, 
            offset, 
            map);

        if (segment_info.zone_paths.empty()) {
            std::cout << "[Create_Local_Problem] WARNING: Agent " << agent_id
                        << " has no path segment inside the extended window" << std::endl;
            continue;
        }
        //find out if the agent was in the zone, if so, also check at the last timestep
        // -> returning agent, if there at last timestep -> continueing agent
        // otherwise new agent
        bool was_in_the_zone = state.original_to_segments.find(agent_id) != state.original_to_segments.end();
        bool was_in_the_zone_at_last_timestep = false;

        //if the agent was in the zone, get the last segment that was in the zone
        if (was_in_the_zone) {
            int last_segment_idx = state.original_to_segments.at(agent_id).back();
            LocalSegment& last_segment = state.segments[last_segment_idx];
            was_in_the_zone_at_last_timestep = last_segment.exit_t >= previous_zone_end_t;
            //continueing agent
            if (was_in_the_zone_at_last_timestep) {
                LocalSegment& segment_to_continue = last_segment;
                if (segment_info.entry_t.empty() || segment_info.exit_t.empty()) {
                    std::cout << "[Create_Local_Problem] ERROR: Missing entry/exit info for continuing agent "
                              << agent_id << std::endl;
                    continue;
                }
                if (segment_info.entry_t.size() != segment_info.exit_t.size()) {
                    std::cout << "[Create_Local_Problem] ERROR: Mismatched entry/exit counts for continuing agent "
                              << agent_id << std::endl;
                    continue;
                }
                if (segment_to_continue.exit_t > state.zone_end_t) {
                    std::cout << "[Create_Local_Problem] ERROR: Agent " << agent_id << "already extended to the end of the zone" << std::endl;
                    continue;
                } else {
                    //std::cout << "[Create_Local_Problem] Agent " << agent_id << " is continuing in the zone" << std::endl;
                    //extend the segment to the end of the first segment in the new window
                    const int old_exit = segment_to_continue.exit_t;
                    const int new_exit = segment_info.exit_t[0];
                    if (new_exit < segment_info.entry_t[0]) {
                        std::cout << "[Create_Local_Problem] ERROR: Segment exit " << new_exit
                                  << " is earlier than entry " << segment_info.entry_t[0]
                                  << " for agent " << agent_id << std::endl;
                        continue;
                    }
                    if (new_exit < old_exit) {
                        std::cout << "[Create_Local_Problem] ERROR: New exit" << new_exit
                                  << " time is less than old exit time" << old_exit
                                  << "for agent " << agent_id << std::endl;
                        continue;
                    }
                    if (new_exit >= static_cast<int>(global_path.size())) {
                        std::cout << "[Create_Local_Problem] ERROR: New exit" << new_exit 
                                  << " time is greater than the end of the path" << global_path.size() -1
                                  << "for agent " << agent_id << std::endl;
                        continue;
                    }
                    const size_t original_path_size = segment_to_continue.path.size();
                    const int required_path_size = new_exit - segment_to_continue.entry_t + 1;
                    if (required_path_size <= 0) {
                        std::cout << "[Create_Local_Problem] ERROR: Required path size" << required_path_size 
                                  << " is less than or equal to 0 for agent " << agent_id << std::endl;
                        continue;
                    }
                    if (static_cast<int>(segment_to_continue.path.size()) < required_path_size) {
                        segment_to_continue.path.resize(required_path_size);
                    }
                    
                    //extend the segment path to the end of the zone by pushing the global path
                    bool extension_valid = true;
                    for (int i = old_exit + 1; i <= new_exit; ++i) {
                        const int local_index = i - segment_to_continue.entry_t;
                        if (local_index < 0 || local_index >= static_cast<int>(segment_to_continue.path.size())) {
                            std::cout << "[Create_Local_Problem] ERROR: Segment path not resized correctly for agent " << agent_id << std::endl;
                            extension_valid = false;
                            continue;
                        }
                        segment_to_continue.path[local_index] = global_path[i];
                       
                        const int relative_index = i - segment_info.entry_t[0];
                        if (relative_index >= 0 && relative_index < static_cast<int>(segment_info.zone_paths[0].size())
                            && global_path[i] != segment_info.zone_paths[0][relative_index]) {
                            std::cout << "[Create_Local_Problem] ERROR: Global path and segment path do not match at time " << i << std::endl;
                            extension_valid = false;
                            continue;
                        }
                    }
                    if (!extension_valid) {
                        std::cout << "[Create_Local_Problem] ERROR: Segment path extension is not valid for agent " << agent_id << std::endl;
                        if (segment_to_continue.path.size() > original_path_size) {
                            segment_to_continue.path.resize(original_path_size);
                            std::cout << "[Create_Local_Problem] Restored segment path to original size for agent " << agent_id << std::endl;
                        }
                        continue;
                    }
                    segment_to_continue.exit_t = new_exit;
                    if (state.zone_end_t < segment_to_continue.exit_t) {
                        std::cout << "[Create_Local_Problem] ERROR: This should not happen. Segment exit time is greater than the end of the zone for agent " << agent_id << std::endl;
                        state.zone_end_t = segment_to_continue.exit_t;
                    }
                    //update the segment accordingly
                    //continueing the segment until the new exit time
                    segment_to_continue.mdd = build_segment_mdd_with_optional_wait_tail(masked_map, 
                                                                                        segment_to_continue.path, 
                                                                                        current_solution.goals[agent_id], 
                                                                                        segment_to_continue.entry_t, 
                                                                                        segment_to_continue.exit_t, 
                                                                                        state.zone_start_t, 
                                                                                        state.zone_end_t, 
                                                                                        agent_id);
                    if (!segment_to_continue.mdd) {
                        std::cout << "[Create_Local_Problem] ERROR: Failed to build MDD for agent " << agent_id << std::endl;
                        continue;
                    }
                    //check if there are any new conflicts in the segment?
                    // -> no we already have all conflicts in the zone
                }
            } else {
                //agent returns
                std::cout << "[Create_Local_Problem] Agent " << agent_id << " is returning to the zone" << std::endl;
                //create new pseudo agent
                int pseudo_agent_id = state.next_pseudo_id++;
                //get new agents path in the zone
                //create new segment
                LocalSegment new_segment;
                new_segment.segment_id = pseudo_agent_id;
                new_segment.original_id = agent_id;
                if (segment_info.entry_t.empty() || segment_info.exit_t.empty()) {
                    std::cout << "[Create_Local_Problem] ERROR: Missing entry/exit info for returning agent "
                              << agent_id << std::endl;
                    continue;
                }
                new_segment.entry_t = segment_info.entry_t[0];
                new_segment.exit_t = segment_info.exit_t[0];
                new_segment.original_entry_t = segment_info.entry_t[0];
                new_segment.original_exit_t = segment_info.exit_t[0];
                new_segment.path = std::move(segment_info.zone_paths[0]);
                //make new mdd
                new_segment.mdd = build_segment_mdd_with_optional_wait_tail(masked_map, 
                                                                            new_segment.path, 
                                                                            current_solution.goals[agent_id], 
                                                                            new_segment.entry_t, 
                                                                            new_segment.exit_t, 
                                                                            state.zone_start_t, 
                                                                            state.zone_end_t, 
                                                                            agent_id);
                if (!new_segment.mdd) {
                    std::cout << "[Create_Local_Problem] ERROR: Failed to build MDD for agent " << agent_id << std::endl;
                    continue;
                }

                //check if there are any new conflicts in the segment?
                // -> no we already have all conflicts in the zone

                state.original_to_pseudo_ids[agent_id].push_back(pseudo_agent_id);
                state.segment_index_by_id[pseudo_agent_id] = state.segments.size();
                state.original_to_segments[agent_id].push_back(state.segments.size());
                state.segments.push_back(new_segment);

            }   
        } else {
            //new agent
            std::cout << "[Create_Local_Problem] Agent " << agent_id << " is new in the zone" << std::endl;
            //create new segment
            LocalSegment new_segment;
            new_segment.segment_id = agent_id;
            new_segment.original_id = agent_id;
            if (segment_info.entry_t.empty() || segment_info.exit_t.empty()) {
                std::cout << "[Create_Local_Problem] ERROR: Missing entry/exit info for new agent "
                          << agent_id << std::endl;
                continue;
            }
            new_segment.entry_t = segment_info.entry_t[0];
            new_segment.exit_t = segment_info.exit_t[0];
            new_segment.original_entry_t = segment_info.entry_t[0];
            new_segment.original_exit_t = segment_info.exit_t[0];
            new_segment.path = std::move(segment_info.zone_paths[0]);
            //make new mdd
            new_segment.mdd = build_segment_mdd_with_optional_wait_tail(masked_map, 
                                                                        new_segment.path, 
                                                                        current_solution.goals[agent_id], 
                                                                        new_segment.entry_t, 
                                                                        new_segment.exit_t, 
                                                                        state.zone_start_t, 
                                                                        state.zone_end_t, 
                                                                        agent_id);
            if (!new_segment.mdd) {
                std::cout << "[Create_Local_Problem] ERROR: Failed to build MDD for agent " << agent_id << std::endl;
                continue;
            }
            state.original_to_segments[agent_id].push_back(state.segments.size());
            state.segment_index_by_id[agent_id] = state.segments.size();
            state.segments.push_back(new_segment);
        }

        //now we took care of the agents first appearance in the extended zone window
        //all subsequent appearances are added as new pseudo agents
        for (int i = 1; i < segment_info.zone_paths.size(); ++i) {
            //create new pseudo agent
            int pseudo_agent_id = state.next_pseudo_id++;
            //create new segment
            LocalSegment new_segment;
            new_segment.segment_id = pseudo_agent_id;
            new_segment.original_id = agent_id;
            new_segment.entry_t = segment_info.entry_t[i];
            new_segment.exit_t = segment_info.exit_t[i];
            new_segment.original_entry_t = segment_info.entry_t[i];
            new_segment.original_exit_t = segment_info.exit_t[i];
            new_segment.path = std::move(segment_info.zone_paths[i]);
            //make new mdd
            new_segment.mdd = build_segment_mdd_with_optional_wait_tail(masked_map, 
                                                                        new_segment.path,
                                                                        current_solution.goals[new_segment.original_id], 
                                                                        new_segment.entry_t, 
                                                                        new_segment.exit_t, 
                                                                        state.zone_start_t, 
                                                                        state.zone_end_t, 
                                                                        new_segment.original_id);
            if (!new_segment.mdd) {
                std::cout << "[Create_Local_Problem] ERROR: Failed to build MDD for agent " << new_segment.original_id << std::endl;
                continue;
            }

            state.original_to_pseudo_ids[agent_id].push_back(pseudo_agent_id);
            state.segment_index_by_id[pseudo_agent_id] = state.segments.size();
            state.original_to_segments[agent_id].push_back(state.segments.size());
            state.segments.push_back(new_segment);
        }

    }
}