#include "Waiting_time_Solve.h"

#include "Create_Local_Problem.h"
#include "Lazy_SAT_Solve.h"
#include "VerificationHelpers.h"
#include "../cnf/CNFConstructor.h"
#include "../mdd/MDDConstructor.h"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <set>
#include <tuple>
#include <unordered_map>


//todos:
//time window extension
//collision validity checking 
// when using waiting time -> rebuild mdd
// when using waiting time -> update following agents mdds to fit the delay

//introduce mdd timed positions to check collision validity -> have segment timed positions --> collision validity checking with timed positions



namespace {

using VertexCollision = std::tuple<int, int, std::pair<int,int>, int>;
using EdgeCollision = std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>;

struct RemovedCollisions {
    std::vector<VertexCollision> vertex;
    std::vector<EdgeCollision> edge;

    bool empty() const { return vertex.empty() && edge.empty(); }
};


using TimedPosition = std::tuple<int, int, int>;
using TimedPositionSet = std::set<TimedPosition>;

TimedPositionSet gather_mdd_timed_positions(const std::shared_ptr<MDD>& mdd) {
    TimedPositionSet result;
    if (!mdd) {
        return result;
    }

    for (const auto& [time, nodes] : mdd->levels) {
        for (const auto& node : nodes) {
            if (!node) {
                continue;
            }
            result.emplace(time, node->position.first, node->position.second);
        }
    }

    return result;
}


bool segment_allows_position_at_time(
    const LocalSegment& segment,
    const std::pair<int,int>& position,
    int absolute_t) {
    if (!segment.mdd) {
        return false;
    }

    auto level_it = segment.mdd->levels.find(absolute_t);
    if (level_it == segment.mdd->levels.end()) {
        return false;
    }

    const auto& nodes = level_it->second;
    return std::any_of(nodes.begin(), nodes.end(), [&](const std::shared_ptr<MDDNode>& node) {
        return node && node->position == position;
    });
}


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

bool remove_collision_from_segment(LocalZoneState& state, const VertexCollision& collision) {
    bool removed = false;
    auto remove_from = [&](int seg_id) {
        auto idx_it = state.segment_index_by_id.find(seg_id);
        if (idx_it == state.segment_index_by_id.end()) {
            return;
        }
        auto& list = state.segments[idx_it->second].vertex_collisions;
        auto it = std::find(list.begin(), list.end(), collision);
        if (it != list.end()) {
            list.erase(it);
            removed = true;
        }
    };
    remove_from(std::get<0>(collision));
    remove_from(std::get<1>(collision));
    return removed;
}

bool remove_collision_from_segment(LocalZoneState& state, const EdgeCollision& collision) {
    bool removed = false;
    auto remove_from = [&](int seg_id) {
        auto idx_it = state.segment_index_by_id.find(seg_id);
        if (idx_it == state.segment_index_by_id.end()) {
            return;
        }
        auto& list = state.segments[idx_it->second].edge_collisions;
        auto it = std::find(list.begin(), list.end(), collision);
        if (it != list.end()) {
            list.erase(it);
            removed = true;
        }
    };
    remove_from(std::get<0>(collision));
    remove_from(std::get<1>(collision));
    return removed;
}

RemovedCollisions filter_collisions(LocalZoneState& state, LocalSegment& segment) {
    RemovedCollisions removed;

    for (const auto& collision : segment.vertex_collisions) {
        int t = std::get<3>(collision);
        const auto& position = std::get<2>(collision);
        bool in_time_window = t >= segment.entry_t && t <= segment.exit_t;
        bool in_mdd = in_time_window && segment_allows_position_at_time(segment, position, t);
        if (!in_mdd) {
            removed.vertex.push_back(collision);
        }
    }
    for (const auto& collision : segment.edge_collisions) {
        int t = std::get<4>(collision);
        const auto& position = std::get<2>(collision);
        const auto& next_position = std::get<3>(collision);
        bool in_time_window = t >= segment.entry_t && (t + 1) <= segment.exit_t;
        bool in_mdd = false;

        if (segment.segment_id == std::get<0>(collision)) {
            in_mdd = in_time_window && segment_allows_position_at_time(segment, position, t) && segment_allows_position_at_time(segment, next_position, t + 1);
        } else {
            in_mdd = in_time_window && segment_allows_position_at_time(segment, next_position, t) && segment_allows_position_at_time(segment, position, t + 1);
        }
        if (!in_mdd) {
            removed.edge.push_back(collision);
        }
    }

    for (const auto& collision : removed.vertex) {
        remove_collision_from_segment(state, collision);
    }
    for (const auto& collision : removed.edge) {
        remove_collision_from_segment(state, collision);
    }

    return removed;
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



//also updates the global solution with the new path
void apply_waiting_time_delta(
    LocalZoneState& state,
    int segment_id,
    int original_id,
    int waiting_time_delta,
    const std::vector<std::vector<char>>& masked_map,
    const std::vector<std::vector<char>>& map,
    CurrentSolution& current_solution,
    std::mt19937& rng) {
    if (waiting_time_delta <= 0) {
        return;
    }

    auto idx_it = state.segment_index_by_id.find(segment_id);
    if (idx_it == state.segment_index_by_id.end()) {
        std::cerr << "[Waiting_time_Solve] ERROR: Segment " << segment_id
                  << " not found while applying waiting time" << std::endl;
        return;
    }

    size_t seg_index = idx_it->second;
    if (seg_index >= state.segments.size()) {
        std::cerr << "[Waiting_time_Solve] ERROR: Segment index " << seg_index
                  << " out of bounds for segment " << segment_id << std::endl;
        return;
    }

    auto align_segment_mdd = [&](LocalSegment& target) {
        if (!target.mdd) {
            std::cerr << "[Waiting_time_Solve] WARNING: Segment " << target.segment_id
                      << " missing MDD; skipping alignment after waiting adjustment" << std::endl;
            return;
        }
        if (target.mdd->levels.empty()) {
            std::cerr << "[Waiting_time_Solve] WARNING: Segment " << target.segment_id
                      << " has empty MDD; skipping alignment after waiting adjustment" << std::endl;
            return;
        }
        align_mdd_to_time_window(
            target.mdd,
            target.entry_t,
            target.exit_t,
            state.zone_start_t,
            state.zone_end_t);
    };

    LocalSegment& segment = state.segments[seg_index];
    if (segment.original_id != original_id) {
        std::cerr << "[Waiting_time_Solve] WARNING: Segment " << segment_id
                  << " original agent mismatch (expected " << original_id
                  << ", got " << segment.original_id << ")" << std::endl;
    }

    const int old_exit = segment.exit_t;
    segment.exit_t += waiting_time_delta;
    state.zone_end_t = std::max(state.zone_end_t, segment.exit_t);

    if (!segment.path.empty()) {
        const auto last_position = segment.path.back();
        for (int i = 0; i < waiting_time_delta; ++i) {
            segment.path.push_back(last_position);
        }
        const auto& start_pos = segment.path.front();
        const auto& goal_pos = segment.path.back();
        int segment_length = segment.exit_t - segment.entry_t + 1;
        if (segment_length <= 0) {
            std::cerr << "[Waiting_time_Solve] ERROR: Segment " << segment_id
                      << " has 0 length path" << std::endl;
            segment_length = static_cast<int>(segment.path.size());
        } 
        MDDConstructor constructor(masked_map, start_pos, goal_pos, std::max(0, segment_length - 1));
        segment.mdd = constructor.construct_mdd();
        segment.path = segment.mdd->sample_random_path(rng); //initial place holder path
    } else {
        std::cerr << "[Waiting_time_Solve] ERROR: Segment " << segment_id
                  << " has no path" << std::endl;
        segment.mdd.reset();
    }

    auto original_it = state.original_to_segments.find(segment.original_id);
    if (original_it == state.original_to_segments.end()) {
        std::cerr << "[Waiting_time_Solve] WARNING: No ordering information for agent "
                  << segment.original_id << " when shifting subsequent segments" << std::endl;
        return;
    }
    const auto& indices = original_it->second;
    auto pos_it = std::find(indices.begin(), indices.end(), seg_index);
    if (pos_it == indices.end()) {
        std::cerr << "[Waiting_time_Solve] WARNING: Segment index " << seg_index
                  << " missing from ordering for agent " << segment.original_id << std::endl;
        return;
    }

    for (auto follow_it = std::next(pos_it); follow_it != indices.end(); ++follow_it) {
        size_t follow_index = *follow_it;
        if (follow_index == seg_index) {
            std::cerr << "[Waiting_time_Solve] ERROR: Shifting segment " << segment_id
                      << " which was extended by waiting time. should only shift segments after the extended one" << std::endl;
            continue;
        }
        if (follow_index >= state.segments.size()) {
            std::cerr << "[Waiting_time_Solve] WARNING: Segment index " << follow_index
                      << " out of range while shifting agent " << segment.original_id << std::endl;
            continue;
        }
        LocalSegment& following = state.segments[follow_index];
        following.entry_t += waiting_time_delta;
        following.exit_t += waiting_time_delta;
        filter_collisions(state, following);
        state.zone_end_t = std::max(state.zone_end_t, following.exit_t);
    }

    //allign mdds for segment and all following segments
    align_segment_mdd(segment);
    for (auto follow_it = std::next(pos_it); follow_it != indices.end(); ++follow_it) {
        size_t follow_index = *follow_it;
        if (follow_index >= state.segments.size()) {
            std::cerr << "[Waiting_time_Solve] WARNING: Segment index " << follow_index
                      << " out of range while shifting agent " << segment.original_id << std::endl;
            continue;
        }
        align_segment_mdd(state.segments[follow_index]);
    }

    //update global solution with the stretched and displaced paths
    //have the stretched segment longer than before need to shift the suffix first
    auto& new_path = current_solution.agent_paths.at(segment.original_id);
    //before the segment entry time, the path is the same
    //after the segment exit time, the path is the same but delayed by the waiting time delta
    for (int i = segment.exit_t + 1; i < static_cast<int>(new_path.size()); ++i) {
        new_path[i] = new_path[i - waiting_time_delta];
    }    
    //during the segment the path is the segment path
    for (int i = segment.entry_t; i <= segment.exit_t; ++i) {
        new_path[i] = segment.path[i - segment.entry_t];
    }
    //now place all following segments into the new path, they do not come with extra delays and can replaced one to one
    for (auto follow_it = std::next(pos_it); follow_it != indices.end(); ++follow_it) {
        size_t follow_index = *follow_it;
        if (follow_index >= state.segments.size()) {
            std::cerr << "[Waiting_time_Solve] WARNING: Segment index " << follow_index
                      << " out of range while shifting agent " << segment.original_id << std::endl;
            continue;
        }
        LocalSegment& following = state.segments[follow_index];
        for (int i = following.entry_t; i <= following.exit_t; ++i) {
            new_path[i] = following.path[i - following.entry_t];
        }
    }
    //verify path validity
    if (!verify_path_consistency(new_path, map)) {
        std::cerr << "[Waiting_time_Solve] ERROR: Path is not consistent after updating with waiting time" << std::endl;
        return;
    }
    //verify start and goal
    if (new_path.front() != current_solution.starts[segment.original_id]) {
        std::cerr << "[Waiting_time_Solve] ERROR: Path does not start at the start position" << std::endl;
        return;
    }
    if (new_path.back() != current_solution.goals[segment.original_id]) {
        std::cerr << "[Waiting_time_Solve] ERROR: Path does not end at the goal position" << std::endl;
        return;
    }
    //puth new path into current solution
    current_solution.agent_paths[segment.original_id] = new_path;
    //Now current solution is updated with the new path
    //we can update the path map
    current_solution.create_path_map();

    if (segment.exit_t != old_exit + waiting_time_delta) {
        std::cerr << "[Waiting_time_Solve] WARNING: Segment " << segment_id
                  << " exit time mismatch after waiting adjustment" << std::endl;
    }
}



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
    int initial_waiting_time_amount,
    std::mt19937& rng) {

    (void)map;
    (void)local_zone_conflict_indices;

    LazySolveResult result;
    result.solution_found = false;
    
    //assert waiting time and make a backup so we can restore it if waiting time solve fails
    auto waiting_time_backup = current_solution.backup_waiting_times();
    auto paths_backup = current_solution.backup_paths();

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
        
        merge_collisions(state, lazy_result.discovered_vertex_collisions, lazy_result.discovered_edge_collisions);
        
        if (lazy_result.solution_found) {
            std::unordered_map<int, std::vector<std::pair<int,int>>> original_paths;
            std::unordered_map<int, std::pair<int,int>> new_entry_exit_time;

            //update local zone state segments paths
            for (auto& segment : state.segments) {
                auto it_path = lazy_result.local_paths.find(segment.segment_id);
                if (it_path == lazy_result.local_paths.end()) {
                    std::cout << "[Waiting_time_Solve] ERROR: Missing path for segment " << segment.segment_id << std::endl;
                    continue;
                }
                segment.path = it_path->second;   
            }
            //update global solution
            //todo: update delayed current solution 
            // when we extend paths in the current solution at the time we deploy waiting time 
            //we need to pay attention on how the local solution is integrated into the global solution
            //we need to make sure that the global solution is updated correctly
            //todo: also check for the correct time adjustments of local segments when deploying waiting time

            current_solution.update_with_local_paths_and_pseudo_agents(state, lazy_result.local_paths);

            result = lazy_result;
            result.local_paths = std::move(lazy_result.local_paths);
            result.local_entry_exit_time = std::move(new_entry_exit_time);
            result.solution_found = true;
            return result;
        }

        auto pending_vertex_collisions = lazy_result.latest_discovered_vertex_collisions;
        auto pending_edge_collisions = lazy_result.latest_discovered_edge_collisions;

        bool applied_wait = false;
        bool extended_time_window = false;
        int previous_zone_end_t = state.zone_end_t;
        //adjust them if we use waiting time
        auto try_apply_wait = [&](int segment_id) {
            auto idx_it = state.segment_index_by_id.find(segment_id);
            if (idx_it == state.segment_index_by_id.end()) return false;
            const LocalSegment& segment = state.segments[idx_it->second];
            int original_id = segment.original_id;
            if (current_solution.get_waiting_time(original_id) < waiting_delta) {
                return false;
            }
            current_solution.use_waiting_time(original_id, waiting_delta);
            //update local zone state segments associated with the agent            
            //TODO updating mdds
            apply_waiting_time_delta(state, segment_id, original_id, waiting_delta, masked_map, map, current_solution, rng);
            //check if we extended the time window
            if (state.zone_end_t > previous_zone_end_t) {
                extended_time_window = true;
            }

            std::cout << "[Waiting_time_Solve] Applied waiting time to segment " << segment_id << " for agent " << original_id << std::endl;

            applied_wait = true;

            return true;
        };

        //use waiting time for the agents with unresolved conflicts
        std::set<int> agents_already_used_waiting_time; //to avoid using waiting time for the same agent twice
        bool out_of_waiting_time = false;
        int agent1 = 0;
        int agent2 = 0;
        for (const auto& collision : pending_vertex_collisions) {
            agent1 = std::get<0>(collision);
            agent2 = std::get<1>(collision);
            //check if we already used waiting time for one of the agents 
            if (agents_already_used_waiting_time.count(agent1) > 0 ||
                agents_already_used_waiting_time.count(agent2) > 0) {
                continue;
            }
            //check which agent has more waiting time and use it
            if (current_solution.get_waiting_time(agent1) >= current_solution.get_waiting_time(agent2)) {
                if (try_apply_wait(agent1)){
                    agents_already_used_waiting_time.insert(agent1);
                } else {
                    out_of_waiting_time = true;
                    break; //impossible to use waiting time
                }
            } else {
                if (try_apply_wait(agent2)){
                    agents_already_used_waiting_time.insert(agent2);
                } else {
                    out_of_waiting_time = true;
                    break; //impossible to use waiting time
                }
            }
        }
        for (const auto& collision : pending_edge_collisions) {
            agent1 = std::get<0>(collision);
            agent2 = std::get<1>(collision);
            //check if we already used waiting time for one of the agents 
            if (agents_already_used_waiting_time.count(agent1) > 0 ||
                agents_already_used_waiting_time.count(agent2) > 0) {
                continue;
            }
            //check which agent has more waiting time and use it
            if (current_solution.get_waiting_time(agent1) >= current_solution.get_waiting_time(agent2)) {
                if (try_apply_wait(agent1)){
                    agents_already_used_waiting_time.insert(agent1);
                } else {
                    out_of_waiting_time = true;
                    break; //impossible to use waiting time
                }
            } else {
                if (try_apply_wait(agent2)){
                    agents_already_used_waiting_time.insert(agent2);
                } else {
                    out_of_waiting_time = true;
                    break; //impossible to use waiting time
                }
            }
        }
        
        
        if (!applied_wait) {
            std::cout << "[Waiting_time_Solve] No waiting time applied" << std::endl;
            break;
        } else {
            std::cout << "[Waiting_time_Solve] Waiting time applied" << std::endl;
            
        }
        if (out_of_waiting_time) {
            std::cout << "[Waiting_time_Solve] Out of waiting time" << std::endl;
            break;
        }
        if (extended_time_window) {
            std::cout << "[Waiting_time_Solve] Extended Zone end time from " << previous_zone_end_t << " to " << state.zone_end_t << std::endl;
            //check for new agents that enter the zone at the new timesteps
           
            refresh_zone_after_extension(
                state, 
                current_solution, 
                local_zone_positions, 
                previous_zone_end_t, 
                masked_map,
                map,
                conflict_map,
                conflict_meta,
                offset);
        }
    }

    if (!result.solution_found) {
        std::cout << "[Waiting_time_Solve] No solution found" << std::endl;
        std::cout << "[Waiting_time_Solve] Restoring original paths" << std::endl;

    }
    std::cout << "[Waiting_time_Solve] Restoring waiting times" << std::endl;
    current_solution.restore_waiting_times(waiting_time_backup);
    current_solution.restore_paths(paths_backup);
    
    return result;
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

//unused
std::pair<std::set<std::pair<int,int>>, bool> choose_agents_to_wait(
    const std::vector<ConflictMeta>& current_conflicts, 
    const CurrentSolution& current_solution,
    std::unordered_map<int, int> original_agent_id) {

    std::set<std::pair<int,int>> agents_to_use_waiting_time_with_pseudo_agents;
    bool can_use_waiting_time = true;
    auto is_agent_already_selected = [&](int original_id, int pseudo_id) {
        return std::any_of(
            agents_to_use_waiting_time_with_pseudo_agents.begin(),
            agents_to_use_waiting_time_with_pseudo_agents.end(),
            [&](const auto& entry) {
                return entry.first == original_id || entry.second == pseudo_id;
            });
    };
    //iterate through all current conflicts -> skip if one of the agents is already chosen, otherwise add the one with more waiting time
    for (const auto& conflict : current_conflicts) {
        int agent_1 = conflict.agent1;
        int agent_1_pseudo = agent_1;
        int agent_2 = conflict.agent2;
        int agent_2_pseudo = agent_2;
        bool agent_1_is_pseudo = false;
        bool agent_2_is_pseudo = false;
        //check if agent is a pseudo agent
        if (auto it = original_agent_id.find(agent_1); it != original_agent_id.end()) {
            agent_1 = it->second;
            agent_1_is_pseudo = true;
        }
        if (auto it = original_agent_id.find(agent_2); it != original_agent_id.end()) {
            agent_2 = it->second;
            agent_2_is_pseudo = true;
        }
        int agent_1_waiting_time = current_solution.get_waiting_time(agent_1);
        int agent_2_waiting_time = current_solution.get_waiting_time(agent_2);
        //does one have waiting time?
        if (agent_1_waiting_time > 0 || agent_2_waiting_time > 0) {
            //if one of the agent is alerady chosen, skip
            if (is_agent_already_selected(agent_1, agent_1_pseudo) ||
                is_agent_already_selected(agent_2, agent_2_pseudo)) {
                continue;
            }
            //add the one with more waiting time
            if (agent_1_waiting_time >= agent_2_waiting_time) {
                auto selected = std::make_pair(agent_1, agent_1_pseudo);
                agents_to_use_waiting_time_with_pseudo_agents.insert(selected);
                if (agent_1_is_pseudo) {
                    std::cout << "[Waiting_time_Solve] Selected pseudo agent " << agent_1_pseudo
                                << " representing Agent " << agent_1 << " for waiting time" << std::endl;
                }
            } else {
                auto selected = std::make_pair(agent_2, agent_2_pseudo);
                agents_to_use_waiting_time_with_pseudo_agents.insert(selected);
                if (agent_2_is_pseudo) {
                    std::cout << "[Waiting_time_Solve] Selected pseudo agent " << agent_2_pseudo
                                << " representing Agent " << agent_2 << " for waiting time" << std::endl;
                }
            }
        }else{
            //if both agents have no waiting time, we can't use waiting time
            can_use_waiting_time = false;
            break; // no need to check other agents
        }
    }
    if (!agents_to_use_waiting_time_with_pseudo_agents.empty()) {
        std::cout << "[Waiting_time_Solve] Agents selected for waiting time:" << std::endl;
        for (const auto& [original_id, pseudo_id] : agents_to_use_waiting_time_with_pseudo_agents) {
            if (original_id == pseudo_id) {
                std::cout << "  - Agent " << original_id << std::endl;
            } else {
                std::cout << "  - Agent " << original_id << " (pseudo agent " << pseudo_id << ")" << std::endl;
            }
        }
    }
    return {agents_to_use_waiting_time_with_pseudo_agents, can_use_waiting_time};
}