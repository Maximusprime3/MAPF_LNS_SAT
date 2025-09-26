#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Current_Solution.h"
#include "MDDConstructor.h"
#include "MDD.h"




void void align_mdd_to_time_window(std::shared_ptr<MDD> mdd,
    int entry_t, int exit_t, //agent's entry and exit times
    int start_t, int end_t) { //time window of problem zone start and end

    std::cout << "[LNS] Starting to align MDD to the time window: [" << start_t << ", " << end_t << "]" << std::endl;
    if (!mdd) { // do we have an mdd?
        std::cout << "[LNS] ERROR: No MDD to align" << std::endl;
        return;
    }

    std::cout << "[LNS] MDD levels: " << mdd->levels.size() << std::endl;
    //is it empty?
    if (mdd->levels.empty()) {
        std::cout << "[LNS] ERROR: MDD is empty" << std::endl;
        return;
    }

    if (end_t < start_t) { // does start and end make sense?
        std::cerr << "[LNS] ERROR: Invalid time window for MDD alignment (end_t < start_t)." << std::endl;
        mdd->levels.clear();
        return;
    }

    int zone_mdd_length = end_t - start_t + 1;
    // Calculate relative timesteps within the time window
    int relative_entry = entry_t - start_t;  // 0-based within the time window
    int relative_exit = exit_t - start_t;    // 0-based within the time window

    relative_entry = std::max(0, std::min(relative_entry, zone_mdd_length - 1));
    //relative_exit = std::max(relative_entry, std::min(relative_exit, zone_mdd_length - 1)); //we don't need to do this as we don't add empty levels after the agent's exit

    auto original_levels = mdd->levels;
    std::map<int, std::vector<std::shared_ptr<MDDNode>>> aligned_levels;

    //check if already aligned to the time window 
    //if first MDD level matches entry
    std::cout << "[LNS] Checking if MDD start is already aligned to the time window" << std::endl;
    bool mdd_start_aligned = false;
    if (original_levels.begin()->first == relative_entry + start_t) {
        std::cout << "[LNS] MDD start " << original_levels.begin()->first << " already at relative entry: " << relative_entry + start_t << std::endl;
        mdd_start_aligned = true;
    }

    //if last MDD level matches exit
    std::cout << "[LNS] Checking if MDD end already at relative exit: " << relative_exit + start_t << std::endl;
    bool mdd_end_aligned = false;
    // get last MDD level 
    if (original_levels.rbegin()->first == relative_exit + start_t) {
        std::cout << "[LNS] MDD end " << original_levels.rbegin()->first << " already aligned to the time window: " << relative_exit + start_t << std::endl;
        mdd_end_aligned = true;
    }

    std::cout << "checked if MDD start and end are aligned" << std::endl;
    if (mdd_start_aligned && mdd_end_aligned) {
        std::cout << "[LNS] MDD already aligned to the time window" << std::endl;
        return;
    }

    // Shift the agent's MDD levels to the correct position
    std::cout << "[LNS] Aligning now" << std::endl;
    for (const auto& [level, nodes] : original_levels) {

        int new_level = level + relative_entry + start_t; //shift the level to the correct position in the window

        if (new_level >= zone_mdd_length + start_t) {
            std::cerr << "[LNS] WARNING: MDD level " << new_level
                      << " exceeds time window length " << zone_mdd_length + start_t
                      << "; truncating." << std::endl;
            continue;
        }

        auto& target_nodes = aligned_levels[new_level]; //get the target nodes
        target_nodes = nodes;
        for (auto& node : target_nodes) {
            node->time_step = new_level; //MDD is now in absolute time scale similar to collision metadata and update logic 
        }
    }

    std::cout << "[LNS] Aligned MDD to the time window" << std::endl;
    mdd->levels = std::move(aligned_levels);
}



// Helper function to extract agent path segments within a zone
struct AgentPathSegment {
    std::vector<std::pair<int,int>> contiguous_intervals;
    int entry_t;
    int exit_t;
    bool agent_in_zone;
};

AgentPathSegment extract_agent_path_segment(
    int agent_id,
    const std::vector<std::pair<int,int>>& path,
    const std::set<std::pair<int,int>>& zone_positions_set,
    int start_t,
    int end_t) {
    
    AgentPathSegment segment;
    segment.entry_t = -1;
    segment.exit_t = -1;
    segment.agent_in_zone = false;
    
    int window_span = end_t - start_t + 1;
    if (window_span > 0) {    
        segment.contiguous_intervals.reserve(window_span);
    }

    bool agent_in_zone = false;
    // Getting path segments in the zone
    for (int t = start_t; t <= end_t && t < (int)path.size(); ++t) {
        const auto& pos = path[t]; // walking along the agent's path
        bool pos_in_zone = zone_positions_set.count(pos) > 0;
        if (pos_in_zone) { // agent is in the zone
            if (!agent_in_zone) { // agent is entering the zone
                segment.agent_in_zone = true;
                agent_in_zone = true;
                if (segment.entry_t == -1) {
                    segment.entry_t = t;
                }
                std::cout << "[Create_Local_problem] Agent " << agent_id << " entered the zone at timestep " << t << std::endl;
                segment.contiguous_intervals.emplace_back(t, t); // start new interval
            } else { // agent is already in the zone
                segment.contiguous_intervals.back().second = t; // last known position in the zone
            }
            segment.exit_t = t; 
        } else if (agent_in_zone) { // agent is leaving the zone
            segment.contiguous_intervals.back().second = segment.exit_t; // end of interval in the zone
            std::cout << "[Create_Local_problem] Agent " << agent_id << " exited the zone at timestep " << t << std::endl;
            agent_in_zone = false;
        }       
    }
    
    if (!segment.contiguous_intervals.empty() && segment.agent_in_zone) {
        segment.contiguous_intervals.back().second = segment.exit_t; 
        std::cout << "[Create_Local_problem] Agent " << agent_id << " exited the zone at timestep " << segment.exit_t << std::endl;
    }
    
    return segment;
}




// helper function to process agent paths and handle zone expansion
struct AgentProcessingResult {
    std::vector<std::vector<std::pair<int,int>>> zone_paths; // paths for the zone (can be more than one if the agent returns to the zone)
    std::vector<int> entry_t;
    std::vector<int> exit_t;
    std::set<std::pair<int,int>> expanded_zone_positions;
    std::vector<int> additional_agents;
    std::vector<ConflictMeta> additional_conflicts;
};
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
    
    AgentProcessingResult result;
    result.entry_t = std::vector<int>();
    result.exit_t = std::vector<int>();
    result.expanded_zone_positions = zone_positions_set;
    
    // Extract agent path segment
    auto segment_info = extract_agent_path_segment(agent_id, path, zone_positions_set, start_t, end_t);
    
    if (!segment_info.contiguous_intervals.empty()) {
        
        if (segment_info.contiguous_intervals.size() > 1) {
            std::cout << "[Create_Local_problem] Agent " << agent_id << " returned to the zone "
                      << (segment_info.contiguous_intervals.size() - 1) << " times within time window" << std::endl;
            std::cout << "[Create_Local_problem] NEED TO HANDLE RETURNING AGENTS AS PSEUDO AGENTS" << std::endl;
        }
        //create seperate paths for the returning agent
        for (const auto& interval : segment_info.contiguous_intervals) {
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


// Helper: Build local zone state (paths, entry/exit, MDDs) for a zone/time window
struct LocalZoneState {
    std::unordered_map<int, std::vector<std::pair<int,int>>> local_zone_paths;
    std::unordered_map<int, std::pair<int,int>> local_entry_exit_time;
    std::unordered_map<int, std::shared_ptr<MDD>> local_mdds;
    std::unordered_map<int, int> original_agent_id; //remember original agent id for pseudo agents (fake id, real id)
};


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
    const std::unordered_map<int, int,int,std::vector<std::pair<int,int>>,int>& agent_to_pseudo_agent_id) // (og_id, segment_entry_t, segment_exit_t, segment_path, pseudo_id)
{
    LocalZoneState state;
    // Find agents present in the zone within time window
    auto agents_in_window = get_agents_in_zone_within_time_window(current_solution, zone_positions_set, start_t, end_t);
    //check how many pseudo agents we have already
    int number_of_pseudo_agents = agent_to_pseudo_agent_id.size();
    const int global_number_of_agents = current_solution.agent_paths.size();
    // Extract segments and compute entry/exit
    for (int agent_id : agents_in_window) {
        const auto& path = current_solution.agent_paths.at(agent_id);
        auto result = process_agent_in_zone(agent_id, path, zone_positions_set, conflict_map, conflict_meta, start_t, end_t, offset, grid);
        if (!result.zone_paths.empty()) {
            //check if only one path
            if (result.zone_paths.size() == 1) {//Agent stays in the zone like a good agent
                state.local_zone_paths[agent_id] = std::move(result.zone_paths[0]);
                state.local_entry_exit_time[agent_id] = std::make_pair(result.entry_t[0], result.exit_t[0]);
            } else if (result.zone_paths.size() > 1) {//Agent leaves and returns to the zone like a bad agent
                std::cout << "[Create_Local_problem] Agent " << agent_id << " has multiple paths in the zone" << std::endl;
                std::cout << "[Create_Local_problem] Need to handle returning agents as PSEUDO AGENTS" << std::endl;
                //need new ids for pseudo agents and create seperate paths for them
                //need to remember who the original agent is 
                for (int i = 1; i < result.zone_paths.size(); i++) { //for each returning path (after the first exit)
                    
                    //create fake id for pseudo agent
                    int pseudo_id = global_number_of_agents + number_of_pseudo_agents;
                    number_of_pseudo_agents++;
                    state.original_agent_id[pseudo_id] = agent_id;
                    state.local_zone_paths[pseudo_id] = std::move(result.zone_paths[i]);
                    state.local_entry_exit_time[pseudo_id] = std::make_pair(result.entry_t[i], result.exit_t[i]);               
                }
            } 
        }else {//Agent has no paths in the zone like a very bad agent
            std::cout << "[Create_Local_problem] ERROR: Agent " << agent_id << " has no paths in the zone" << std::endl;
        }
    }
    // Build MDDs and align to window
    for (const auto& [agent_id, local_path] : state.local_zone_paths) {
        if (agent_id > global_number_of_agents) {
            std::cout << "[Create_Local_problem] Creating pseudo Agent " << agent_id << " is a pseudo agent of Agent " << state.original_agent_id[agent_id] << std::endl;
        }
        auto [entry_t, exit_t] = state.local_entry_exit_time[agent_id];
        auto zone_start_pos = local_path.front();
        auto zone_goal_pos = local_path.back();
        int agent_path_length = exit_t - entry_t + 1;
        MDDConstructor constructor(masked_map, zone_start_pos, zone_goal_pos, agent_path_length - 1);
        auto agent_mdd = constructor.construct_mdd();
        align_mdd_to_time_window(agent_mdd, entry_t, exit_t, start_t, end_t);
        state.local_mdds[agent_id] = agent_mdd;
    }
    return state;
}