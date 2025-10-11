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

//[Current_Solution] ERROR: Segment 2 local path length (1) does not match expected length (9)
// entry exit time not updated? ->rebuild mdds?

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


int count_goal_tail(const std::vector<std::pair<int,int>>& path,
    const std::pair<int,int>& goal) {
    int tail = 0;
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        if (*it == goal) {
            ++tail;
        } else {
            break;
        }
    }
    return tail;
}

bool trim_segment_tail(LocalZoneState& state,
    LocalSegment& segment,
    int trim_amount,
    const std::vector<std::vector<char>>& masked_map,
    const CurrentSolution& current_solution) {
    if (trim_amount <= 0) {
        return true;
    }
    if (segment.path.size() <= static_cast<size_t>(trim_amount)) {
        std::cout << "[Waiting_time_Solve] ERROR: Cannot trim " << trim_amount
                  << " steps from segment " << segment.segment_id
                  << " with path size " << segment.path.size() << std::endl;
        return false;
    }

    const auto goal = current_solution.goals[segment.original_id];
    int available_tail = count_goal_tail(segment.path, goal);
    if (available_tail < trim_amount) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment " << segment.segment_id
                  << " only has " << available_tail
                  << " timesteps of goal tail; cannot trim " << trim_amount << std::endl;
        return false;
    }

    segment.path.resize(segment.path.size() - trim_amount);
    segment.exit_t -= trim_amount;
    if (segment.original_exit_t >= 0) {
        segment.original_exit_t = std::max(segment.original_exit_t - trim_amount, segment.original_entry_t);
    }
    if (segment.exit_t < segment.entry_t) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment " << segment.segment_id
                  << " exit time " << segment.exit_t
                  << " earlier than entry " << segment.entry_t
                  << " after trimming" << std::endl;
        return false;
    }

    state.zone_end_t = std::max(state.zone_end_t, segment.exit_t);

    if (segment.path.empty()) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment " << segment.segment_id
                  << " path empty after trimming" << std::endl;
        return false;
    }

    segment.mdd = build_segment_mdd_with_optional_wait_tail(masked_map,
                                                            segment.path,
                                                            current_solution.goals[segment.original_id],
                                                            segment.entry_t,
                                                            segment.exit_t,
                                                            state.zone_start_t,
                                                            state.zone_end_t,
                                                            segment.original_id);
    if (!segment.mdd) {
        std::cout << "[Waiting_time_Solve] ERROR: Failed to rebuild MDD for segment "
                  << segment.segment_id << " after trimming" << std::endl;
        return false;
    }

    return true;
}

bool can_apply_waiting_time_delta(const LocalZoneState& state,
    int segment_id,
    int original_id,
    int waiting_time_delta,
    const CurrentSolution& current_solution) {
    if (waiting_time_delta <= 0) {
        return true;
    }

    auto idx_it = state.segment_index_by_id.find(segment_id);
    if (idx_it == state.segment_index_by_id.end()) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment " << segment_id
                  << " not found while checking if waiting time delta can be applied" << std::endl;
        return false;
    }

    size_t seg_index = idx_it->second;
    if (seg_index >= state.segments.size()) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment index " << seg_index
                  << " out of bounds for segment " << segment_id << std::endl;
        return false;
    }

    const LocalSegment& segment = state.segments[seg_index];
    if (segment.original_id != original_id) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment " << segment_id
                  << " original id mismatch with original id " << original_id << std::endl;
        return false;
    }

    int remaining_wait = waiting_time_delta;
    const auto global_goal_pos = current_solution.goals[original_id];
    auto goal_it = std::find(segment.path.begin(), segment.path.end(), global_goal_pos);
    if (goal_it != segment.path.end()) {
        int idx = static_cast<int>(std::distance(segment.path.begin(), goal_it));
        int absolute_idx_time = segment.entry_t + idx;
        int segment_waiting_slack = segment.exit_t - absolute_idx_time;
        if (segment_waiting_slack < 0) {
            segment_waiting_slack = 0;
        }
        if (segment_waiting_slack >= remaining_wait) {
            return true;
        }
        remaining_wait -= segment_waiting_slack;
    }

    if (remaining_wait <= 0) {
        return true;
    }

    auto order_it = state.original_to_segments.find(original_id);
    if (order_it == state.original_to_segments.end() || order_it->second.empty()) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment " << segment_id
                  << " original id not found in original to segments" << std::endl;
        return false;
    }

    size_t last_index = order_it->second.back();
    if (last_index >= state.segments.size()) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment index " << last_index
                  << " out of bounds for segment " << segment_id << std::endl;
        return false;
    }

    int path_last_index = static_cast<int>(current_solution.agent_paths.at(original_id).size()) - 1;
    const LocalSegment& last_segment = state.segments[last_index];
    int prospective_last_exit = last_segment.exit_t + remaining_wait;
    if (last_index == seg_index) {
        prospective_last_exit = segment.exit_t + remaining_wait;
    }

    int overflow = prospective_last_exit - path_last_index;
    if (overflow <= 0) {
        return true;
    }

    int available_tail = count_goal_tail(last_segment.path, current_solution.goals[original_id]);
    return available_tail >= overflow;
}




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
bool apply_waiting_time_delta(
    LocalZoneState& state,
    int segment_id,
    int original_id,
    int waiting_time_delta,
    const std::vector<std::vector<char>>& masked_map,
    const std::vector<std::vector<char>>& map,
    CurrentSolution& current_solution,
    std::mt19937& rng) {
    if (waiting_time_delta <= 0) {
        return true;
    }

    int amount_of_waiting_time = waiting_time_delta;
    std::cout << "[Waiting_time_Solve] Applying waiting time delta " << amount_of_waiting_time << " to segment " << segment_id << std::endl;
    std::cout << "[Waiting_time_Solve] Original id: " << original_id << std::endl;
    //verify current solution for consistency
    if (!verify_path_consistency(current_solution.agent_paths[original_id], map)) {
        std::cout << "[Waiting_time_Solve] ERROR: Current solution wrong, before applying waiting time" << std::endl;
        return false;
    }

    auto idx_it = state.segment_index_by_id.find(segment_id);
    if (idx_it == state.segment_index_by_id.end()) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment " << segment_id
                  << " not found while applying waiting time" << std::endl;
        return false;
    }

    size_t seg_index = idx_it->second;
    if (seg_index >= state.segments.size()) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment index " << seg_index
                  << " out of bounds for segment " << segment_id << std::endl;
        return false;
    }

    auto align_segment_mdd = [&](LocalSegment& target) {
        if (!target.mdd) {
            std::cout << "[Waiting_time_Solve] ERROR: Segment " << target.segment_id
                      << " missing MDD; skipping alignment after waiting adjustment" << std::endl;
            return;
        }
        if (target.mdd->levels.empty()) {
            std::cout << "[Waiting_time_Solve] ERROR: Segment " << target.segment_id
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
        std::cout << "[Waiting_time_Solve] WARNING: Segment " << segment_id
                  << " original agent mismatch (expected " << original_id
                  << ", got " << segment.original_id << ")" << std::endl;
    }

    const int old_exit = segment.exit_t;
    bool need_to_apply_waiting_time_delta = true;
    //if the segment already containts the waiting tail, no need to extend the segment exit time but to move the tail
    //if more movement is needed than there is tail in the segment, extend the segment exit time
    const auto global_goal_pos = current_solution.goals[segment.original_id];
    const auto it = std::find(segment.path.begin(), segment.path.end(), global_goal_pos);
    if (it != segment.path.end()) {
        const int idx = static_cast<int>(std::distance(segment.path.begin(), it));

        const int absolute_idx_time = segment.entry_t + idx;
        int segment_waiting_slack = segment.exit_t - absolute_idx_time;
        if (segment_waiting_slack < 0) {
            segment_waiting_slack = 0;
        }
        if (segment_waiting_slack >= amount_of_waiting_time) {
            //there is enough room to apply waiting time delta inside the segment without extending the segment exit time
            //new first goal idx
            int new_first_goal_idx = idx + amount_of_waiting_time;
            //move the tail beginning back without extending the segment exit time
            segment.mdd = build_segment_mdd_with_optional_wait_tail(masked_map, 
                                                                    segment.path,
                                                                    current_solution.goals[segment.original_id], 
                                                                    segment.entry_t, 
                                                                    segment.exit_t, 
                                                                    state.zone_start_t, 
                                                                    state.zone_end_t, 
                                                                    segment.original_id,
                                                                    new_first_goal_idx);
            need_to_apply_waiting_time_delta = false;
            amount_of_waiting_time = 0;
        } else {
            //else we need to extend the segment exit time but we can also move the tail beginning back
            //by doing so we already apply some of the waiting time delta
            int usable_waiting_time_in_segment = segment_waiting_slack;
            int new_first_goal_idx = idx + usable_waiting_time_in_segment; //remove the tail

            segment.mdd = build_segment_mdd_with_optional_wait_tail(masked_map, 
                                                                        segment.path,
                                                                        current_solution.goals[segment.original_id], 
                                                                        segment.entry_t, 
                                                                        segment.exit_t, 
                                                                        state.zone_start_t, 
                                                                        state.zone_end_t, 
                                                                        segment.original_id,
                                                                        new_first_goal_idx);
            amount_of_waiting_time -= usable_waiting_time_in_segment;
        }
    }
    if (need_to_apply_waiting_time_delta) {
        //first extend the segment exit time
        segment.exit_t += amount_of_waiting_time;
        state.zone_end_t = std::max(state.zone_end_t, segment.exit_t);

        if (!segment.path.empty()) {
            const auto last_position = segment.path.back();
            for (int i = 0; i < amount_of_waiting_time; ++i) {
                segment.path.push_back(last_position);
            }
            int segment_length = segment.exit_t - segment.entry_t + 1;
            if (segment_length <= 0) {
                std::cout << "[Waiting_time_Solve] ERROR: Segment " << segment_id
                        << " has 0 length path" << std::endl;
                segment_length = static_cast<int>(segment.path.size());
            } 
            segment.mdd = build_segment_mdd_with_optional_wait_tail(masked_map, 
                                                                    segment.path, 
                                                                    current_solution.goals[segment.original_id], 
                                                                    segment.entry_t, 
                                                                    segment.exit_t, 
                                                                    state.zone_start_t, 
                                                                    state.zone_end_t, 
                                                                    segment.original_id);
            if (!segment.mdd) {
                std::cout << "[Waiting_time_Solve] ERROR: Failed to build MDD for agent " << segment.original_id << std::endl;
                return false;
            }

            //segment.path = segment.mdd->sample_random_path(rng); //initial place holder path
        } else {
            std::cout << "[Waiting_time_Solve] ERROR: Segment " << segment_id
                    << " has no path" << std::endl;
            segment.mdd.reset();
        }
    }

    auto original_it = state.original_to_segments.find(segment.original_id);
    if (original_it == state.original_to_segments.end()) {
        std::cout << "[Waiting_time_Solve] ERROR: No ordering information for agent "
                  << segment.original_id << " when shifting subsequent segments" << std::endl;
        return false;
    }
    const auto& indices = original_it->second;
    auto pos_it = std::find(indices.begin(), indices.end(), seg_index);
    if (pos_it == indices.end()) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment index " << seg_index
                  << " missing from ordering for agent " << segment.original_id << std::endl;
        return false;
    }

    const auto& agent_path_reference = current_solution.agent_paths.at(segment.original_id);
    if (agent_path_reference.empty()) {
        std::cout << "[Waiting_time_Solve] ERROR: Agent path is empty before shifting segments" << std::endl;
        return false;
    }
    const int path_last_index = static_cast<int>(agent_path_reference.size()) - 1;
    int trimmed_from_segment = 0;

    for (auto follow_it = std::next(pos_it); follow_it != indices.end(); ++follow_it) {
        size_t follow_index = *follow_it;
        if (follow_index == seg_index) {
            std::cout << "[Waiting_time_Solve] ERROR: Shifting segment " << segment_id
                      << " which was extended by waiting time. should only shift segments after the extended one" << std::endl;
            return false;
        }
        if (follow_index >= state.segments.size()) {
            std::cout << "[Waiting_time_Solve] ERROR: Segment index " << follow_index
                      << " out of range while shifting agent " << segment.original_id << std::endl;
            return false; 
        }
        LocalSegment& following = state.segments[follow_index];
        following.entry_t += amount_of_waiting_time;
        following.exit_t += amount_of_waiting_time;
        RemovedCollisions removed = filter_collisions(state, following);
        if (!removed.empty()) {
            std::cout << "[Waiting_time_Solve] Pruned "
                      << removed.vertex.size() << " vertex and "
                      << removed.edge.size() << " edge collisions from segment "
                      << following.segment_id << std::endl;
        }
        state.zone_end_t = std::max(state.zone_end_t, following.exit_t);

        bool is_last_segment = (std::next(follow_it) == indices.end());
        if (is_last_segment) {
            int overflow = following.exit_t - path_last_index;
            if (overflow > 0) {
                if (!trim_segment_tail(state, following, overflow, masked_map, current_solution)) {
                    return false;
                }
            }
        }
    }

    if (std::next(pos_it) == indices.end()) {
        int overflow = segment.exit_t - path_last_index;
        if (overflow > 0) {
            if (!trim_segment_tail(state, segment, overflow, masked_map, current_solution)) {
                return false;
            }
            trimmed_from_segment = overflow;
        }
    }

    //allign mdds for segment and all following segments
    align_segment_mdd(segment);
    for (auto follow_it = std::next(pos_it); follow_it != indices.end(); ++follow_it) {
        size_t follow_index = *follow_it;
        if (follow_index >= state.segments.size()) {
            std::cout << "[Waiting_time_Solve] WARNING: Segment index " << follow_index
                      << " out of range while shifting agent " << segment.original_id << std::endl;
            continue;
        }
        align_segment_mdd(state.segments[follow_index]);
    }
    
    //update global solution with the stretched and displaced paths
    //have the stretched segment longer than before need to shift the suffix first
    auto& new_path = current_solution.agent_paths.at(segment.original_id);
    if (new_path.empty()) {
        std::cout << "[Waiting_time_Solve] ERROR: Agent path is empty before applying waiting time" << std::endl;
        return false;
    }
    //safety check if the path ends at the goal
    if (!verify_path_consistency(new_path, map)) {
        std::cout << "[Waiting_time_Solve] ERROR: Path is not consistent before updating with waiting time" << std::endl;
        return false;
    }
    if (new_path.back() != current_solution.goals[segment.original_id]) {
        std::cout << "[Waiting_time_Solve] ERROR: Path does not end at the goal before updating with waiting time" << std::endl;
        return false;
    }
    
    //path can be longer than before, need to find last segment exit_t and resize path if needed


    //before the segment entry time, the path is the same
    //after the segment exit time, the path is the same but delayed by the waiting time delta
    const int path_length = static_cast<int>(new_path.size());
    //print path length and makespan
    std::cout << "[Waiting_time_Solve] Path length: " << path_length << std::endl;
    std::cout << "[Waiting_time_Solve] Makespan: " << current_solution.max_timestep << std::endl;
    std::cout << "[Waiting_time_Solve] Segment exit time: " << segment.exit_t << std::endl;
    const int suffix_start = static_cast<int>(std::min(segment.exit_t + 1, path_length - 1));
    for (int i = path_length - 1; i >= suffix_start; --i) {
        int src = i - amount_of_waiting_time;
        if (src < 0) {
            src = 0;
        }
        new_path[i] = new_path[src];
    }
    std::cout << "[Waiting_time_Solve] Path length after shifting suffix: " << new_path.size() << std::endl;
    //check goal
    if (new_path.back() != current_solution.goals[segment.original_id]) {
        std::cout << "[Waiting_time_Solve] ERROR: made room for the segment, now path does not end at the goal" << std::endl;
        //print waiting time delta
        std::cout << "[Waiting_time_Solve] Waiting time delta: " << amount_of_waiting_time << std::endl;
        //print agent waiting time
        std::cout << "[Waiting_time_Solve] Agent waiting time: " << current_solution.get_waiting_time(segment.original_id) << std::endl;
        //print path
        std::cout << "[Waiting_time_Solve] Path: ";
        for (const auto& pos : new_path) {
            std::cout << "(" << pos.first << ", " << pos.second << ") ";
        }
        std::cout << std::endl;
    }
    //during the segment the path is the segment path
    for (int i = segment.entry_t; i <= segment.exit_t; ++i) {
        new_path[i] = segment.path[i - segment.entry_t];
    }
    std::cout << "[Waiting_time_Solve] path length after inserting segment path: " << new_path.size() << std::endl;
    //check goal
    if (new_path.back() != current_solution.goals[segment.original_id]) {
        std::cout << "[Waiting_time_Solve] ERROR: placed the segment, now path does not end at the goal" << std::endl;
    }
    std::cout << "[Waiting_time_Solve] inserting following segments" << std::endl;
    std::cout << "[Waiting_time_Solve] curren pos_it: " << *pos_it << std::endl;
    //if we have next pos_it, print it
    if (std::next(pos_it) != indices.end()) {
        std::cout << "[Waiting_time_Solve] next pos_it: " << *(std::next(pos_it)) << std::endl;
    }

    //now place all following segments into the new path, they do not come with extra delays and can replaced one to one
    for (auto follow_it = std::next(pos_it); follow_it != indices.end(); ++follow_it) {
        size_t follow_index = *follow_it;
        if (follow_index >= state.segments.size()) {
            std::cout << "[Waiting_time_Solve] WARNING: Segment index " << follow_index
                      << " out of range while shifting agent " << segment.original_id << std::endl;
            continue;
        }
        std::cout << "[Waiting_time_Solve] inserting following segment at index " << follow_index << std::endl;
        LocalSegment& following = state.segments[follow_index];
        for (size_t i = 0; i < following.path.size(); ++i) {
            std::cout << "[Waiting_time_Solve] inserting following segment path at time " << following.entry_t + i << std::endl;
            std::cout << "[Waiting_time_Solve] following segment path: " << following.path[i].first << ", " << following.path[i].second << std::endl;
            new_path[following.entry_t + i] = following.path[i];
        }
        std::cout << "[Waiting_time_Solve] path length after inserting following segment: " << new_path.size() << std::endl;
    }
    //verify path validity
    if (!verify_path_consistency(new_path, map)) {
        std::cout << "[Waiting_time_Solve] ERROR: Path is not consistent after updating with waiting time" << std::endl;
    }
    //does it end at the goal?
    if (new_path.back() != current_solution.goals[segment.original_id]) {
        std::cout << "[Waiting_time_Solve] ERROR: Path does not end at the goal after placing all following segments" << std::endl;
    }
    //verify start and goal
    if (new_path.front() != current_solution.starts[segment.original_id]) {
        std::cout << "[Waiting_time_Solve] ERROR: Path does not start at the start position" << std::endl;
    }
    if (new_path.back() != current_solution.goals[segment.original_id]) {
        std::cout << "[Waiting_time_Solve] ERROR: Path does not end at the goal position" << std::endl;
    }
    //check if the path is consistent
    if (!verify_path_consistency(new_path, map)) {
        std::cout << "[Waiting_time_Solve] ERROR: Applying waiting time, then path is not consistent" << std::endl;
        if (verify_path_consistency(current_solution.agent_paths[segment.original_id], map)) {
            std::cout << "[Waiting_time_Solve] ERROR: Global solution is consistent, but replacement is not after applying waiting time" << std::endl;
        }
    }

    //verify global solution for consistency
    if (!verify_path_consistency(current_solution.agent_paths[segment.original_id], map)) {
        std::cout << "[Waiting_time_Solve] ERROR: Current solution wrong, before pushing new path " << segment.original_id << std::endl;
    }
    std::cout << "[Waiting_time_Solve] pushing new path into current solution" << std::endl;
    //puth new path into current solution
    current_solution.agent_paths[segment.original_id] = new_path;

    //verify global solution for consistency
    if (!verify_path_consistency(current_solution.agent_paths[segment.original_id], map)) {
        std::cout << "[Waiting_time_Solve] ERROR: Current solution wrong, after pushing new path " << segment.original_id << std::endl;
    }
    //verify new path for consistency
    if (!verify_path_consistency(new_path, map)) {
        std::cout << "[Waiting_time_Solve] ERROR: New path is not consistent after applying waiting time" << std::endl;
    } else {
        std::cout << "[Waiting_time_Solve] New path is consistent after applying waiting time for agent " << segment.original_id << std::endl;
    }

    //Now current solution is updated with the new path
    //we can update the path map
    current_solution.create_path_map();

    if (segment.exit_t != old_exit + amount_of_waiting_time - trimmed_from_segment) {
        std::cout << "[Waiting_time_Solve] ERROR: Segment " << segment_id
                  << " exit time mismatch after waiting adjustment" << std::endl;
        return false;
    }
    return true;
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
   
    LocalZoneState baseline_state = state;
    
    auto original_entry_exit = build_original_entry_exit_time_map(state);

    // first iteration should be 0 waiting time
    int waiting_delta = 0;
    if (initial_waiting_time_amount > 0) {
        waiting_delta = initial_waiting_time_amount;
    }
    

    
    //const int max_iterations = 100;
    //total available waiting time
    int total_available_waiting_time = 0;

    for (const auto& [agent_id, waiting_time] : current_solution.agent_waiting_time) {
        total_available_waiting_time += waiting_time;
    }
    std::cout << "[Waiting_time_Solve] Total available waiting time: " << total_available_waiting_time << std::endl;
    const int max_iterations = total_available_waiting_time;
    for (int iter = 0; iter < max_iterations; iter++) {
        std::cout << "[Waiting_time_Solve] Iteration " << iter << "..." << std::endl;

        

        auto mdd_map = build_segment_mdd_map(state);
        CNFConstructor cnf_constructor(mdd_map, true);
        CNF local_cnf = cnf_constructor.construct_cnf();

        // Before constructing the SAT instance, purge any collisions whose
        // timestamps no longer align with the updated segment MDDs. Without
        // this, stale collisions can reference timesteps that the MDD no
        // longer supports, leading to missing CNF variables (e.g., pseudo
        // agents appearing one timestep off in the logs).
        for (size_t seg_idx = 0; seg_idx < state.segments.size(); ++seg_idx) {
            auto& segment = state.segments[seg_idx];
            auto removed = filter_collisions(state, segment);
            if (!removed.empty()) {
                std::cout << "[Waiting_time_Solve] Pruned "
                          << removed.vertex.size() << " vertex and "
                          << removed.edge.size() << " edge collisions from segment "
                          << segment.segment_id << std::endl;
            }
        }

        //add collision clauses to cnf
        auto cached_vertex_collisions = gather_vertex_collisions(state);
        auto cached_edge_collisions = gather_edge_collisions(state);


        auto entry_exit_map = build_segment_entry_exit_time_map(state);

        //print all agents with all corresponding pseudo agents in one line per agent
        for (const auto& [original_id, pseudo_ids] : state.original_to_pseudo_ids) {
            if (pseudo_ids.empty()) {
                continue;
            }
            std::cout << "[Waiting_time_Solve] Agent " << original_id << " pseudo agents: ";
            for (const auto& pseudo_id : pseudo_ids) {
                std::cout << pseudo_id << " ";
            }
            std::cout << std::endl;
        }

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
            std::cout << "[Waiting_time_Solve] Solution found, updating global solution" << std::endl;
            //update local zone state segments paths
            for (auto& segment : state.segments) {
                auto it_path = lazy_result.local_paths.find(segment.segment_id);
                if (it_path == lazy_result.local_paths.end()) {
                    std::cout << "[Waiting_time_Solve] ERROR: Missing path for segment " << segment.segment_id << std::endl;
                    continue;
                }
                segment.path = it_path->second;   
            }

            //verify local solution for consistency
            for (const auto& segment : state.segments) {
                if (!verify_path_consistency(segment.path, map)) {
                    std::cout << "[Waiting_time_Solve] ERROR: Local solution is not consistent after updating with local paths" << std::endl;
                }
            }

            //verify current solution before updating
            for (const auto& [agent_id, path] : current_solution.agent_paths) {
                if (!verify_path_consistency(path, map)) {
                    std::cout << "[Waiting_time_Solve] ERROR: Current solution is not consistent before updating with local paths" << std::endl;
                }
                int waiting_time = current_solution.get_waiting_time(agent_id);
                if (path[path.size() - waiting_time - 1] != current_solution.goals[agent_id]) {
                    std::cout << "[Waiting_time_Solve] ERROR: before updating, Agent " << agent_id << " does not end at the goal position with waiting time" << std::endl;
                }
                if (path.back() != current_solution.goals[agent_id]) {
                    std::cout << "[Waiting_time_Solve] ERROR: before updating, Agent " << agent_id << " does not end at the goal position" << std::endl;
                }

            }

            //update global solution
            //todo: update delayed current solution 
            // when we extend paths in the current solution at the time we deploy waiting time 
            //we need to pay attention on how the local solution is integrated into the global solution
            //we need to make sure that the global solution is updated correctly
            //todo: also check for the correct time adjustments of local segments when deploying waiting time

            current_solution.update_with_local_paths_and_pseudo_agents(state, lazy_result.local_paths, map);

            //verify every agent has their amount of waiting time as goal positions in the end of their path
            for (const auto& [agent_id, path] : current_solution.agent_paths) {
                if (path.back() != current_solution.goals[agent_id]) {
                    std::cout << "[LNS] ERROR: Agent " << agent_id << " does not end at the goal position" << std::endl;
                }
                int waiting_time = current_solution.get_waiting_time(agent_id);
                if (path[path.size() - waiting_time - 1] != current_solution.goals[agent_id]) {
                    std::cout << "[Waiting_time_Solve] ERROR: SOLUTION FOUND BUT Agent " << agent_id << " does not end at the goal position with waiting time" << std::endl;
                    //print waiting time
                    std::cout << "[Waiting_time_Solve] Waiting time: " << waiting_time << std::endl;
                    //print path
                    std::cout << "[Waiting_time_Solve] Path (size: " << path.size() << "): ";
                    for (const auto& pos : path) {
                        std::cout << "(" << pos.first << ", " << pos.second << ") ";
                    }
                    std::cout << std::endl;
                }
            }

            result = lazy_result;
            result.local_paths = std::move(lazy_result.local_paths);
            result.local_entry_exit_time = std::move(new_entry_exit_time);
            result.solution_found = true;
            return result;
        }
        //check lazy result solution for consistency
        for (const auto& path : lazy_result.local_paths) {
            if (!verify_path_consistency(path.second, map)) {
                std::cout << "[Waiting_time_Solve] ERROR: Lazy result path is not consistent for agent" << path.first << std::endl;
                //get og id
                int original_id = state.segments[state.segment_index_by_id.find(path.first)->second].original_id;
                std::cout << "[Waiting_time_Solve] ERROR: Lazy result path is not consistent for agent " << original_id << std::endl;
                return result;
            }
        }

        //increment waiting time
        waiting_delta++;
        std::cout << "[Waiting_time_Solve] No solution found, trying to apply waiting time:" << waiting_delta << std::endl;
        auto pending_vertex_collisions = lazy_result.latest_discovered_vertex_collisions;
        auto pending_edge_collisions = lazy_result.latest_discovered_edge_collisions;

        bool applied_wait = false;
        bool extended_time_window = false;
        int previous_zone_end_t = state.zone_end_t;
        //adjust them if we use waiting time
        auto try_apply_wait = [&](int segment_id, int amount_of_waiting_time) {
            auto idx_it = state.segment_index_by_id.find(segment_id);
            if (idx_it == state.segment_index_by_id.end()) return false;
            const LocalSegment& segment = state.segments[idx_it->second];
            int original_id = segment.original_id;
            if (current_solution.get_waiting_time(original_id) < amount_of_waiting_time) {
                std::cout << "[Waiting_time_Solve] ERROR: Agent " << original_id << " has not enough waiting time to apply " << amount_of_waiting_time << std::endl;
                return false;
            }
            //print waiting time delta
            //std::cout << "[Waiting_time_Solve] Applying " << amount_of_waiting_time << " waiting time to agent " << original_id << std::endl;
            //print agent waiting time
            //std::cout << "[Waiting_time_Solve] Agent waiting time: " << current_solution.get_waiting_time(original_id) << std::endl;
            //current makespan
            //std::cout << "[Waiting_time_Solve] Current makespan: " << current_solution.max_timestep << std::endl;
            //print path
            //std::cout << "[Waiting_time_Solve] Path (size: " << current_solution.agent_paths[original_id].size() << "): ";
            //for (const auto& pos : current_solution.agent_paths[original_id]) {
            //    std::cout << "(" << pos.first << ", " << pos.second << ") ";
            //}
            //std::cout << std::endl;

            if (!can_apply_waiting_time_delta(state, segment_id, original_id, amount_of_waiting_time, current_solution)) {
                std::cout << "[Waiting_time_Solve] ERROR: Cannot apply waiting time delta " << amount_of_waiting_time
                          << " to segment " << segment_id << " without exceeding path length" << std::endl;
                return false;
            }

            //check if the path of this agent is consitent before apllying waiting time
            if (!verify_path_consistency(current_solution.agent_paths[original_id], map)) {
                std::cout << "[Waiting_time_Solve] ERROR: Path is not consistent before applying waiting time to agent " << original_id << "segment " << segment_id << std::endl;
                return false;
            }
            //update local zone state segments associated with the agent            
            if (!apply_waiting_time_delta(state, segment_id, original_id, amount_of_waiting_time, masked_map, map, current_solution, rng)) {
                return false;
            }
            current_solution.use_waiting_time(original_id, amount_of_waiting_time);
            //check if the path of this agent is consitent after apllying waiting time
            if (!verify_path_consistency(current_solution.agent_paths[original_id], map)) {
                std::cout << "[Waiting_time_Solve] ERROR: Path is not consistent after applying waiting time to agent " << original_id << "segment " << segment_id << std::endl;
                return false;
            }
            //check if we extended the time window
            if (state.zone_end_t > previous_zone_end_t) {
                extended_time_window = true;
            }

            std::cout << "[Waiting_time_Solve] Applied " << amount_of_waiting_time << " waiting time to segment " << segment_id << " for agent " << original_id << std::endl;

            applied_wait = true;

            return true;
        };

        //use waiting time for the agents with unresolved conflicts
        std::set<int> agents_already_used_waiting_time; // original agent ids that already consumed waiting time
        bool out_of_waiting_time = false;
        int agent1 = 0;
        int agent2 = 0;
        auto get_original_agent_id = [&](int segment_id) -> std::optional<int> {
            auto idx_it = state.segment_index_by_id.find(segment_id);
            if (idx_it != state.segment_index_by_id.end()) {
                int original_id = state.segments[idx_it->second].original_id;
                if (original_id >= 0 && original_id < static_cast<int>(current_solution.goals.size())) {
                    return original_id;
                }
                std::cout << "[Waiting_time_Solve] WARNING: Original agent id " << original_id
                          << " out of bounds while resolving segment " << segment_id << std::endl;
                return std::nullopt;
            }
            if (segment_id >= 0 && segment_id < static_cast<int>(current_solution.goals.size())) {
                // Already an original agent id
                return segment_id;
            }
            std::cout << "[Waiting_time_Solve] WARNING: Unable to resolve segment " << segment_id
                      << " to a valid original agent" << std::endl;
            return std::nullopt;
        };
        auto get_agent_path = [&](int original_id) -> const std::vector<std::pair<int,int>>* {
            auto path_it = current_solution.agent_paths.find(original_id);
            if (path_it == current_solution.agent_paths.end()) {
                std::cout << "[Waiting_time_Solve] WARNING: Missing path for agent " << original_id
                          << " when attempting to use waiting time" << std::endl;
                return nullptr;
            }
            return &path_it->second;
        };
        auto apply_waiting_time_for_goal_collision = [&](int segment_of_concern,
                                                         int original_of_concern,
                                                         int other_segment,
                                                         int other_original,
                                                         const std::pair<int,int>& collision_pos) -> bool {
            const auto* path_ptr = get_agent_path(original_of_concern);
            if (!path_ptr) {
                return false;
            }
            const auto& path = *path_ptr;
            auto goal = current_solution.goals[original_of_concern];
            auto goal_it = std::find(path.begin(), path.end(), goal);
            auto collision_it = std::find(path.begin(), path.end(), collision_pos);
            if (goal_it == path.end() || collision_it == path.end()) {
                std::cout << "[Waiting_time_Solve] WARNING: Unable to locate goal or collision position in path for agent "
                          << original_of_concern << std::endl;
                return false;
            }
            int idx_of_first_goal = static_cast<int>(std::distance(path.begin(), goal_it));
            int idx_of_collision = static_cast<int>(std::distance(path.begin(), collision_it));
            int waiting_time_needed = std::max(0, idx_of_collision - idx_of_first_goal);

            int current_agents_left_over_waiting_time = current_solution.get_waiting_time(original_of_concern) - waiting_time_needed;
            int other_agents_left_over_waiting_time = current_solution.get_waiting_time(other_original) - waiting_delta;

            if (current_agents_left_over_waiting_time > other_agents_left_over_waiting_time) {
                if (try_apply_wait(segment_of_concern, waiting_time_needed)) {
                    agents_already_used_waiting_time.insert(original_of_concern);
                    return true;
                }
                return false;
            }
            if (try_apply_wait(other_segment, waiting_delta)) {
                agents_already_used_waiting_time.insert(other_original);
                return true;
            }
            return false;
        };
        for (const auto& collision : pending_vertex_collisions) {
            agent1 = std::get<0>(collision);
            agent2 = std::get<1>(collision);
            auto original_agent1 = get_original_agent_id(agent1);
            auto original_agent2 = get_original_agent_id(agent2);
            if (!original_agent1 || !original_agent2) {
                continue;
            }
            //check if we already used waiting time for one of the agents 
            if (agents_already_used_waiting_time.count(*original_agent1) > 0 ||
                agents_already_used_waiting_time.count(*original_agent2) > 0) {
                continue;
            }

            //spceial case collision happens at the global goal positin of either agent
            std::pair<int,int> collision_pos = std::get<2>(collision);
            const auto& goal1 = current_solution.goals[*original_agent1];
            const auto& goal2 = current_solution.goals[*original_agent2];
            if (collision_pos == goal1 || collision_pos == goal2) {
                int segment_of_concern = (collision_pos == goal1) ? agent1 : agent2;
                int original_of_concern = (segment_of_concern == agent1) ? *original_agent1 : *original_agent2;
                int other_segment = (segment_of_concern == agent1) ? agent2 : agent1;
                int other_original = (segment_of_concern == agent1) ? *original_agent2 : *original_agent1;
                if (!apply_waiting_time_for_goal_collision(segment_of_concern,
                                                           original_of_concern,
                                                           other_segment,
                                                           other_original,
                                                           collision_pos)) {
                    out_of_waiting_time = true;
                    break;
                }
                continue;
            }

            //check which agent has more waiting time and use it
            if (current_solution.get_waiting_time(*original_agent1) >= current_solution.get_waiting_time(*original_agent2)) {
                if (try_apply_wait(agent1, waiting_delta)){
                    agents_already_used_waiting_time.insert(*original_agent1);
                } else {
                    out_of_waiting_time = true;
                    break; //impossible to use waiting time
                }
            } else {
                if (try_apply_wait(agent2, waiting_delta)){
                    agents_already_used_waiting_time.insert(*original_agent2);
                } else {
                    out_of_waiting_time = true;
                    break; //impossible to use waiting time
                }
            }
        }
        for (const auto& collision : pending_edge_collisions) {
            agent1 = std::get<0>(collision);
            agent2 = std::get<1>(collision);
            auto original_agent1 = get_original_agent_id(agent1);
            auto original_agent2 = get_original_agent_id(agent2);
            if (!original_agent1 || !original_agent2) {
                continue;
            }
            //check if we already used waiting time for one of the agents
            if (agents_already_used_waiting_time.count(*original_agent1) > 0 ||
                agents_already_used_waiting_time.count(*original_agent2) > 0) {
                continue;
            }

            //spceial case collision happens at the global goal positin of either agent
            std::pair<int,int> collision_pos1 = std::get<2>(collision);
            std::pair<int,int> collision_pos2 = std::get<3>(collision);
            const auto& goal1 = current_solution.goals[*original_agent1];
            const auto& goal2 = current_solution.goals[*original_agent2];
            if (collision_pos1 == goal1 || collision_pos1 == goal2 ||
                collision_pos2 == goal1 || collision_pos2 == goal2) {
                std::pair<int,int> collision_pos = (collision_pos1 == goal1 || collision_pos1 == goal2) ? collision_pos1 : collision_pos2;
                int segment_of_concern = (collision_pos == goal1) ? agent1 : agent2;
                int original_of_concern = (segment_of_concern == agent1) ? *original_agent1 : *original_agent2;
                int other_segment = (segment_of_concern == agent1) ? agent2 : agent1;
                int other_original = (segment_of_concern == agent1) ? *original_agent2 : *original_agent1;
                if (!apply_waiting_time_for_goal_collision(segment_of_concern,
                                                           original_of_concern,
                                                           other_segment,
                                                           other_original,
                                                           collision_pos)) {
                    out_of_waiting_time = true;
                    break;
                }
                continue;
            }
            
            //check which agent has more waiting time and use it
            if (current_solution.get_waiting_time(*original_agent1) >= current_solution.get_waiting_time(*original_agent2)) {
                if (try_apply_wait(agent1, waiting_delta)){
                    agents_already_used_waiting_time.insert(*original_agent1);
                } else {
                    out_of_waiting_time = true;
                    break; //impossible to use waiting time
                }
            } else {
                if (try_apply_wait(agent2, waiting_delta)){
                    agents_already_used_waiting_time.insert(*original_agent2);
                } else {
                    out_of_waiting_time = true;
                    break; //impossible to use waiting time
                }
            }
        }
        
        //check local solution for consistency
        for (const auto& segment : state.segments) {
            if (!verify_path_consistency(segment.path, map)) {
                std::cout << "[Waiting_time_Solve] ERROR: Path is not consistent for segment " << segment.segment_id << std::endl;
                return result;
            }
        }
        if (!applied_wait) {
            std::cout << "[Waiting_time_Solve] No waiting time applied" << std::endl;
            break;
        } else {
            std::cout << "[Waiting_time_Solve] Waiting time applied" << std::endl;
            //verify every agent has their amount of waiting time as goal positions in the end of their path
            for (const auto& [agent_id, path] : current_solution.agent_paths) {
                if (path.back() != current_solution.goals[agent_id]) {
                    std::cout << "[Waiting_time_Solve] ERROR: Agent " << agent_id << " does not end at the goal position" << std::endl;
                }
                int waiting_time = current_solution.get_waiting_time(agent_id);
                if (path[path.size() - waiting_time - 1] != current_solution.goals[agent_id]) {
                    std::cout << "[Waiting_time_Solve] ERROR: APPLIED WAITING TIME BUT Agent " << agent_id << " does not end at the goal position with waiting time" << std::endl;
                    //print waiting time
                    std::cout << "[Waiting_time_Solve] Waiting time: " << waiting_time << std::endl;
                    //print path
                    std::cout << "[Waiting_time_Solve] Path (size: " << path.size() << "): ";
                    for (const auto& pos : path) {
                        std::cout << "(" << pos.first << ", " << pos.second << ") ";
                    }
                    std::cout << std::endl;
                }
            }
        }
        if (out_of_waiting_time) {
            std::cout << "[Waiting_time_Solve] Out of waiting time" << std::endl;
            break;
        }
        if (extended_time_window) {
            std::cout << "[Waiting_time_Solve] Extended Zone end time from " << previous_zone_end_t << " to " << state.zone_end_t << std::endl;

            //verify that the global solution is consistent
            for (const auto& [agent_id, path] : current_solution.agent_paths) {
                if (!verify_path_consistency(path, map)) {
                    std::cout << "[Waiting_time_Solve] ERROR: Global solution is not consistent before refreshing zone for agent " << agent_id << std::endl;
                }
            }
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

            //verify every agent has their amount of waiting time as goal positions in the end of their path
            for (const auto& [agent_id, path] : current_solution.agent_paths) {
                if (path.back() != current_solution.goals[agent_id]) {
                    std::cout << "[Waiting_time_Solve] ERROR: Agent " << agent_id << " does not end at the goal position" << std::endl;
                }
                int waiting_time = current_solution.get_waiting_time(agent_id);
                if (path[path.size() - waiting_time - 1] != current_solution.goals[agent_id]) {
                    std::cout << "[Waiting_time_Solve] ERROR: EXTENDED ZONE END TIME BUT Agent " << agent_id << " does not end at the goal position with waiting time" << std::endl;
                    //print waiting time
                    std::cout << "[Waiting_time_Solve] Waiting time: " << waiting_time << std::endl;
                    //print path
                    std::cout << "[Waiting_time_Solve] Path (size: " << path.size() << "): ";
                    for (const auto& pos : path) {
                        std::cout << "(" << pos.first << ", " << pos.second << ") ";
                    }
                    std::cout << std::endl;
                }
            }

        }
    }

    if (!result.solution_found) {
        std::cout << "[Waiting_time_Solve] No solution found" << std::endl;
        std::cout << "[Waiting_time_Solve] One last try without tailed mdds" << std::endl;
        bool there_are_tailed_mdds = false;
        for (const auto& segment : state.segments) {
            //RESTART WHOLE waiting time solve without using tails ever
            //update waiting time current solution accordingly
            //check if the segment has a tailed mdd
            //check by checking if the path reaches the global goal position
            if (segment.path.back() == current_solution.goals[segment.original_id]) {
                //get tail position -> thats the waiting time we will use
                int tail_idx = std::distance(segment.path.begin(), std::find(segment.path.begin(), segment.path.end(), current_solution.goals[segment.original_id]));
                //its more than one position ahead of the exit time
                if (tail_idx < segment.exit_t) {
                    there_are_tailed_mdds = true;
                    //make a new mdd from entry to exit time
                    MDDConstructor constructor(masked_map, segment.path.front(), segment.path.back(), segment.exit_t - segment.entry_t + 1);
                    auto mdd = constructor.construct_mdd();
                    if (!mdd) {
                        std::cout << "[Waiting_time_Solve] ERROR: Failed to construct mdd for segment " << segment.segment_id << std::endl;
                        return result;
                    }
                    segment.mdd = mdd;
                    //update waiting time used, end_t-tail_idx
                }
            }
        }
        if (there_are_tailed_mdds) {
            std::cout << "[Waiting_time_Solve] There are tailed mdds" << std::endl;
            //SAT solve the local zone
            //if successful, return the result
            //if not try giving all waiting time?
            
        }

        //restore original paths and waiting times
        std::cout << "[Waiting_time_Solve] Restoring original paths" << std::endl;
        std::cout << "[Waiting_time_Solve] Restoring waiting times" << std::endl;
        current_solution.restore_waiting_times(waiting_time_backup);
        current_solution.restore_paths(paths_backup);
    }
    

    //state = baseline_state;
    
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