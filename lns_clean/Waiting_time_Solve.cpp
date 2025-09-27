#include "Waiting_time_Solve.h"

#include "Create_Local_Problem.h"
#include "lazy_SAT_Solve.h"
#include "../cnf/CNFConstructor.h"

#include <algorithm>
#include <iostream>
#include <set>
#include <tuple>
#include <unordered_map>

namespace {

std::vector<std::tuple<int, int, std::pair<int,int>, int>> gather_vertex_collisions(
    const LocalZoneState& state) {
    std::set<std::tuple<int, int, std::pair<int,int>, int>> unique;
    for (const auto& segment : state.segments) {
        for (const auto& collision : segment.vertex_collisions) {
            unique.insert(collision);
        }
    }
    return {unique.begin(), unique.end()};
}
std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> gather_edge_collisions(
    const LocalZoneState& state) {
    std::set<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> unique;
    for (const auto& segment : state.segments) {
        for (const auto& collision : segment.edge_collisions) {
            unique.insert(collision);
        }
    }
    return {unique.begin(), unique.end()};
}

void record_collision(LocalZoneState& state,
    const std::tuple<int, int, std::pair<int,int>, int>& collision) {
    int a1 = std::get<0>(collision);
    int a2 = std::get<1>(collision);
    auto add_to_segment = [&](int seg_id) {
        auto it = state.segment_index_by_id.find(seg_id);
        if (it == state.segment_index_by_id.end()) return;
        auto& list = state.segments[it->second].vertex_collisions;
        if (std::find(list.begin(), list.end(), collision) == list.end()) {
            list.push_back(collision);
        }
    };
    add_to_segment(a1);
    add_to_segment(a2);
}
void record_collision(LocalZoneState& state,
    const std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>& collision) {
    int a1 = std::get<0>(collision);
    int a2 = std::get<1>(collision);
    auto add_to_segment = [&](int seg_id) {
        auto it = state.segment_index_by_id.find(seg_id);
        if (it == state.segment_index_by_id.end()) return;
        auto& list = state.segments[it->second].edge_collisions;
        if (std::find(list.begin(), list.end(), collision) == list.end()) {
            list.push_back(collision);
        }
    };
    add_to_segment(a1);
    add_to_segment(a2);
}

void filter_collisions(LocalSegment segment){
    segment.vertex_collisions.erase(
        std::remove_if(segment.vertex_collisions.begin(), segment.vertex_collisions.end(),
            [&](const auto& collision) {
                int t = std::get<3>(collision);
                return t < segment.entry_t || t > segment.exit_t;
            }),
        segment.vertex_collisions.end());
    segment.edge_collisions.erase(
        std::remove_if(segment.edge_collisions.begin(), segment.edge_collisions.end(),
            [&](const auto& collision) {
                int t = std::get<4>(collision);
                return t < segment.entry_t || t > segment.exit_t;
            }),
        segment.edge_collisions.end());
}

std::unordered_map<int, std::pair<int,int>> build_original_entry_exit_time_map(
    const LocalZoneState& state) {
    std::unordered_map<int, std::pair<int,int>> result;
    for (const auto& segment : state.segments) {
        auto it = result.find(segment.original_id);
        if (it == result.end()) {
            result[segment.original_id] = {segment.original_entry_t, segment.original_exit_t};
        } else {
            it->second.first = std::min(it->second.first, segment.original_entry_t);
            it->second.second = std::max(it->second.second, segment.original_exit_t);
        }
    }
    return result;
}

void merge_collisions(LocalZoneState& state,
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& vertex_collisions,
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& edge_collisions) {
    for (const auto& collision : vertex_collisions) {
        record_collision(state, collision);
    }
    for (const auto& collision : edge_collisions) {
        record_collision(state, collision);
    }
}

}//namespace

std::pair<std::set<int>, bool> choose_agents_to_use_waiting_time(
    const std::vector<ConflictMeta>& current_conflicts, 
    const CurrentSolution& current_solution) {

    std::set<int> agents_to_use_waiting_time;
    bool can_use_waiting_time = true;
    //iterate through all current conflicts -> skip if one of the agents is already chosen, otherwise add the one with more waiting time
    for (const auto& conflict : current_conflicts) {
        int agent_1 = conflict.agent1;
        int agent_2 = conflict.agent2;
        int agent_1_waiting_time = current_solution.get_waiting_time(agent_1);
        int agent_2_waiting_time = current_solution.get_waiting_time(agent_2);
        //does one have waiting time?
        if (agent_1_waiting_time > 0 || agent_2_waiting_time > 0) {
            //if one of the agent is alerady chosen, skip
            if (agents_to_use_waiting_time.count(agent_1) > 0 || agents_to_use_waiting_time.count(agent_2) > 0) {
                continue;
            }
            //add the one with more waiting time
            if (agent_1_waiting_time >= agent_2_waiting_time) {
                agents_to_use_waiting_time.insert(agent_1);
            } else {
                agents_to_use_waiting_time.insert(agent_2);
            }
        }else{
            //if both agents have no waiting time, we can't use waiting time
            can_use_waiting_time = false;
            break; // no need to check other agents
        }
    }
    return {agents_to_use_waiting_time, can_use_waiting_time};
}

std::pair<std::set<int,int>, bool> choose_agents_to_wait(
    const std::vector<ConflictMeta>& current_conflicts, 
    const CurrentSolution& current_solution,
    std::unordered_map<int, int> original_agent_id) {

    std::set<std::pair<int,int>> agents_to_use_waiting_time_with_pseudo_agents;
    bool can_use_waiting_time = true;
    //iterate through all current conflicts -> skip if one of the agents is already chosen, otherwise add the one with more waiting time
    for (const auto& conflict : current_conflicts) {
        int agent_1 = conflict.agent1;
        int agent_1_pseudo = agent_1;
        int agent_2 = conflict.agent2;
        int agent_2_pseudo = agent_2;
        bool agent_1_is_pseudo = false;
        bool agent_2_is_pseudo = false;
        //check if agent is a pseudo agent
        if (original_agent_id.count(agent_1) > 0) {
            agent_1 = original_agent_id.at(agent_1);
            agent_1_is_pseudo = true;
        }
        if (original_agent_id.count(agent_2) > 0) {
            agent_2 = original_agent_id.at(agent_2);
            agent_2_is_pseudo = true;
        }
        int agent_1_waiting_time = current_solution.get_waiting_time(agent_1);
        int agent_2_waiting_time = current_solution.get_waiting_time(agent_2);
        //does one have waiting time?
        if (agent_1_waiting_time > 0 || agent_2_waiting_time > 0) {
            //if one of the agent is alerady chosen, skip
            if (agents_to_use_waiting_time_with_pseudo_agents.count(agent_1,agent_1_pseudo) > 0 || 
                agents_to_use_waiting_time_with_pseudo_agents.count(agent_2,agent_2_pseudo) > 0) {
                continue;
            }
            //add the one with more waiting time
            if (agent_1_waiting_time >= agent_2_waiting_time) {
                agents_to_use_waiting_time_with_pseudo_agents.insert(agent_1,agent_1_pseudo);
            } else {
                agents_to_use_waiting_time_with_pseudo_agents.insert(agent_2,agent_2_pseudo);
            }
        }else{
            //if both agents have no waiting time, we can't use waiting time
            can_use_waiting_time = false;
            break; // no need to check other agents
        }
    }
    return {agents_to_use_waiting_time_with_pseudo_agents, can_use_waiting_time};
}





LazySolveResult lazy_solve_with_waiting_time(
    CurrentSolution& current_solution,
    const std::vector<std::vector<char>>& map,
    const std::vector<std::vector<char>>& masked_map,
    const std::set<std::pair<int,int>>& local_zone_positions,
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<int>& local_zone_conflict_indices,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    int start_t,
    int end_t,
    int offset,
    int initial_waiting_time_amount) {

    (void)map;
    (void)local_zone_conflict_indices;

    LazySolveResult result;
    result.solution_found = false;
    
    //assert waiting time and make a backup so we can restore it if waiting time solve fails
    auto waiting_time_backup = current_solution.backup_waiting_times();

    int using_waiting_time = initial_waiting_time_amount; 

    //std::vector<ConflictMeta> current_conflicts;
    //for (int conflict_idx : local_zone_conflict_indices) {
    //    if (conflict_idx >= 0 && conflict_idx < (int)conflict_meta.size()) {
    //        current_conflicts.push_back(conflict_meta[conflict_idx]);
    //    }
    //}
    
    //Choose agents from initial conflicts to use waiting time and check if we can use waiting time
    //auto [agents_to_use_waiting_time, can_use_waiting_time] = choose_agents_to_use_waiting_time(current_conflicts, current_solution);

    //create the local problem 
    LocalZoneState state = build_local_problem_for_zone(
        current_solution,
        local_zone_positions,
        masked_map,
        map,
        conflict_map,
        conflict_meta,
        offset,
        start_t,
        end_t);
   

    std::unordered_map<int, std::vector<std::pair<int,int>>> agent_path_backup;
    for (const auto& [original_id, _] : state.original_to_segments) {
        auto it = current_solution.agent_paths.find(original_id);
        if (it != current_solution.agent_paths.end()) {
            agent_path_backup[original_id] = it->second;
        }
    }
    auto original_entry_exit = build_original_entry_exit_time_map(state);

    //TODO:what if first iteration should be 0 waiting time?
    int waiting_delta = std::max(1, initial_waiting_time_amount>0? initial_waiting_time_amount : 1);

    
    const int max_iterations = 10;
    for (int iter = 0; iter < max_iterations; iter++) {
        std::cout << "[Waiting_time_Solve] Iteration " << iter << "..." << std::endl;

        auto mdd_map = build_segment_mdd_map(state);
        CNFConstructor cnf_constructor(mdd_map, true);
        CNF local_cnf = cnf_constructor.construct_cnf();

        //add collision clauses to cnf
        auto cached_vertex_collisions = gather_vertex_collisions(state);
        auto cached_edge_collisions = gather_edge_collisions(state);
        if (!cached_vertex_collisions.empty()) {
            cnf_constructor.add_collision_clauses_to_cnf(local_cnf, cached_vertex_collisions);
        }
        if (!cached_edge_collisions.empty()) {
            cnf_constructor.add_edge_collision_clauses_to_cnf(local_cnf, cached_edge_collisions);
        }

        auto entry_exit_map = build_segment_entry_exit_time_map(state);

        //try lazy sat solve
        auto lazy_result = lazy_SAT_solve(
            local_cnf,
            cnf_constructor,
            entry_exit_map,
            state.zone_start_t,
            state.zone_end_t,
            1000, // max_iterations
            cached_vertex_collisions,
            cached_edge_collisions);
        
        merge_collisions(state, lazy_sat_result.discovered_vertex_collisions, lazy_sat_result.discovered_edge_collisions);
        
        if (lazy_sat_result.solution_found) {
            std::unordered_map<int, std::vector<std::pair<int,int>>> original_paths;
            std::unordered_map<int, std::pair<int,int>> new_entry_exit_time;
            for (const auto& segment : state.segments) {
                auto it_path = lazy_result.local_paths.find(segment.segment_id);
                if (it_path != lazy_result.local_paths.end()) continue;
                original_paths[segment.original_id] = it_path->second;
                new_entry_exit_time[segment.original_id] = {segment.entry_t, segment.exit_t};
            }
            current_solution.update_with_local_paths_waiting(original_paths, original_entry_exit, new_entry_exit_time);

            result = lazy_result;
            result.local_paths = std::move(original_paths);
            result.local_entry_exit_time = std::move(new_entry_exit_time);
            result.solution_found = true;
            return result;
        }

        auto pending_vertex_collisions = lazy_result.latest_discovered_vertex_collisions;
        auto pending_edge_collisions = lazy_result.latest_discovered_edge_collisions;

        bool applied_wait = false;
        //adjust them if we use waiting time
        auto try_apply_wait = [&](int segment_id) {
            auto idx_it = state.segment_index_by_id.find(segment_id);
            if (idx_it == state.segment_index_by_id.end()) return false;
            const LocalSegment& segment = state.segments[idx_it->second];
            int original_id = segment.original_id;
            if (current_solution.get_waiting_time(original_id) <= waiting_delta) {
                return false;
            }
            current_solution.use_waiting_time(original_id, waiting_delta);
            apply_waiting_time_delta(state, segment_id, waiting_delta, masked_map);

            std::cout << "[Waiting_time_Solve] Applied waiting time to segment " << segment_id << " for agent " << original_id << std::endl;

            std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths{{original_id, state.segments[idx_it->second].path}};
            std::unordered_map<int, std::pair<int,int>> old_times{{original_id, original_entry_exit[original_id]}};
            std::unordered_map<int, std::pair<int,int>> new_times{{original_id, {state.segments[idx_it->second].entry_t, state.segments[idx_it->second].exit_t}}};

            current_solution.update_with_local_paths_waiting(local_paths, old_times, new_times);
            original_entry_exit[original_id] = new_times[original_id];

            applied_wait = true;

            return true;
        };

        for (const auto& collision : pending_vertex_collisions) {
            if (try_apply_wait(std::get<0>(collision))) break;
            if (try_apply_wait(std::get<1>(collision))) break;
        }
        for (const auto& collision : pending_edge_collisions) {
            if (try_apply_wait(std::get<0>(collision))) break;
            if (try_apply_wait(std::get<1>(collision))) break;
        }
        
        if (!applied_wait) {
            std::cout << "[Waiting_time_Solve] No waiting time applied" << std::endl;
            break;
        }
    }

    if (!result.solution_found) {
        std::cout << "[Waiting_time_Solve] No solution found" << std::endl;
        for (const auto& [agent_id, path] : agent_path_backup) {
            current_solution.agent_paths[agent_id] = path;
        }
    }
    current_solution.restore_waiting_times(waiting_time_backup);
    return result;
}



//BELOW ORIGINAL CODE

    //this was at line 245
    //initialize start and end time, old_end_t is the end time of the previous iteration
    const int zone_start_t = start_t; //zone start time never changes
    int old_end_t = end_t;
    int new_end_t = end_t;

    bool solution_found = false;
    while(!solution_found && can_use_waiting_time){
        

        


        

        //build cnf 
        CNFConstructor cnf_constructor(local_zone_state.local_mdds, true);
        CNF local_cnf = cnf_constructor.construct_cnf();

        //add collision clauses to cnf  
        //WHAT ABOUT PSEUDO AGENTS? -> shoulc be ok since we keep the same ones until we have solution
        //when recreating the local zone we need to make sure the pseudo agents of an agent get the same pseudo agent id as before
        //then we can keep track of all collisions
        //otherwise we can only use the initial collisions
        cnf_constructor.add_collision_clauses_to_cnf(local_cnf, );
        cnf_constructor.add_edge_collision_clauses_to_cnf(local_cnf, );

        //Try lazy SAT solve the local zone
        auto lazy_sat_result = lazy_SAT_solve(
            local_cnf,
            cnf_constructor,
            local_zone_state.local_entry_exit_time,
            zone_start_t, new_end_t,
            10000, // max_iterations
            vertex_collisions,
            edge_collisions);

        if (lazy_sat_result.solution_found) {
            solution_found = true;
            std::cout << "[Waiting_time_Solve] Successfully solved local zone with waiting time" << std::endl;
            //update global solution
            //care for pseudo agents
            
        }
        //keep track of all discovered collisions, might need to adjust them if we use waiting time
        all_discovered_vertex_collisions = lazy_sat_result.discovered_vertex_collisions;
        all_discovered_edge_collisions = lazy_sat_result.discovered_edge_collisions;

        //current conflicts these determine usage of waiting time
        latest_discovered_vertex_collisions = lazy_sat_result.latest_discovered_vertex_collisions;
        latest_discovered_edge_collisions = lazy_sat_result.latest_discovered_edge_collisions;
        //lets get all agents with unresolved conflicts to determine usage of waiting time for these conflicts
        new_conflict_meta = collect_conflicts_meta(latest_discovered_vertex_collisions, latest_discovered_edge_collisions);
        
        //decide which agents to use waiting time for these conflicts
        auto [chosen_agents_with_pseudo_agents, can_use_waiting_time] = choose_agents_to_wait(new_conflict_meta, current_solution, local_zone_state.original_agent_id);
        
        //if impossible to use waiting time, break
        if (!can_use_waiting_time) {
            break;
        }

        //use waiting time for the chosen agents
        //get set of original agent ids
        std::set<int> original_agent_ids;
        bool active_pseudo_agents = false;
        bool extended_time_window = false;
        for (const auto& [agent_id, pseudo_agent_id] : chosen_agents_with_pseudo_agents) {
            if (pseudo_agent_id != agent_id) {
                active_pseudo_agents = true;
            }

            auto entry_exit = local_zone_state.local_entry_exit_time.find(agent_id);
            if (entry_exit == local_zone_state.local_entry_exit_time.end()) {
                std::cerr << "[Waiting_time_Solve] ERROR: Missing entry/exit info for Agent " << agent_id << std::endl;
                continue;
            }
            int entry_t = entry_exit.first;
            int exit_t = entry_exit.second;
            //get path
            auto id_and_path = local_zone_state.local_zone_paths.find(agent_id);
            if (id_and_path == local_zone_state.local_zone_paths.end() || id_and_path->second.empty()) {
                std::cerr << "[Waiting_time_Solve] ERROR: Missing path for Agent " << agent_id << std::endl;
                continue;
            }
            auto& local_path = id_and_path->second;
            
            //use waiting time for the agent
            current_solution.use_waiting_time(agent_id, using_waiting_time);

            //expand MDD by waiting time
            std::cout << "[Waiting_time_Solve] Expanding MDD for Agent " << agent_id << " by " << using_waiting_time << " timestep" << std::endl;

            //get zone start and goal positions
            auto zone_start_pos = local_path.front();
            auto zone_goal_pos = local_path.back();

            //calculate new path length
            int original_path_length = exit_t - entry_t + 1;
            int new_path_length = original_path_length + using_waiting_time;
            int new_exit_t = exit_t + using_waiting_time;

            //create new MDD
            MDDConstructor constructor(masked_map, zone_start_pos, zone_goal_pos, new_path_length - 1);
            auto expanded_mdd = constructor.construct_mdd();

            //TODO: if this agent has pseudo agents after it we need to adjust their entry exit time

            //did we extend the time window?
            if (new_exit_t > new_end_t) {
                new_end_t = new_exit_t; //update new end time
                extended_time_window = true;
            }
            //TODO: also check if we moved any later pseudo agents 

            //update local zone state
            local_zone_state.local_zone_paths[agent_id] = local_path;
            local_zone_state.local_entry_exit_time[agent_id] = std::make_pair(entry_t, new_exit_t);
            local_zone_state.local_mdds[agent_id] = expanded_mdd;

        }
        using_waiting_time += 1;

        //TODO: if we extended the time window, we need to scan the latest timesteps
            //continueing agents
            //new agents
            //returning agents

        //TODO: if we moved any later pseudo agents, we need to check the collisions, if they are still valid
            // some agents might get their timewindow moved so they dont collide anymore --> remove those collisions from cnf creation

        //TODO align local mdds to the time window

        //lazy solve

        
    }
        

        //update local zone state 
        //where are the unresolved conflicts?
        // which agents which segments?
        // there we deploy waiting time
        // update those agents segments
        // did we push final end time of the zone ?
        // if so we need to update the local zone state
        // from previous end t to the new end t 
            //new agents can come in
            //old agents can continue their path
            //old agents can return to the zone -> need new pseudo agents


        //better efficiency if we can use collisions for pseudo agents
        //need to check if pseud agents already existed to recreate them?
        //problem more than one pseudo agent and possibly different extended path
        //or maybe make them more stable to last beyond on solve
        
        

        //when using wating time on an agent with pseudo agents or on a pseudo agent
        //all later pseudo agents need to be updated with entry exit time 
        // probably need to create working copies of current solution and conflict meta
        // and keep track of delays in agents, propagating through the whole path
        // maybe need to build zone problem once and then update correctly with the waiting time usage


        std::cout << "[Waiting_time_Solve] Local zone state: " << local_zone_state.local_zone_paths.size() << " agents" << std::endl;
        std::cout << "[Waiting_time_Solve] number of pseudo agents: " << local_zone_state.original_agent_id.size() << std::endl;

        //update local_zone_paths
        local_zone_paths = local_zone_state.local_zone_paths;
        local_entry_exit_time = local_zone_state.local_entry_exit_time;
        local_mdds = local_zone_state.local_mdds;
        original_agent_id = local_zone_state.original_agent_id;


        //update current conflicts -> latest discovered collisions
        // handle pseudo agents conflicts

        //update which agents to use waiting time for the next iteration
        [agents_to_use_waiting_time, can_use_waiting_time] = choose_agents_to_use_waiting_time(current_conflicts, current_solution);
        //increase waiting time amount
        using_waiting_time += 1;
        // use that waiting time
        current_solution.use_waiting_time(agents_to_use_waiting_time, using_waiting_time);
        //increase exit time for the agents that use waiting time

        //if new exit time of an agent is greater than new_end_t, update new_end_t

        //

    //handle pseudo agents
    //if fail restore waiting time



    }
    return lazy_result;
}