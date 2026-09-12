#include "lnssat/Logging.h"
#include "lnssat/Current_Solution.h"
#include "lnssat/Create_Local_Problem.h"
#include "lnssat/VerificationHelpers.h"
#include <set>

//used globally
//adds edges twice for incex consistency, bc they have two conflictpoints
std::pair<std::vector<std::pair<int,int>>, std::vector<ConflictMeta>> collect_conflicts(
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& vertex_collisions,
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& edge_collisions) {
    std::vector<std::pair<int,int>> conflict_points;
    std::vector<ConflictMeta> conflict_meta ;
    conflict_points.reserve(vertex_collisions.size() + edge_collisions.size() * 2);
    conflict_meta.reserve(vertex_collisions.size() + edge_collisions.size() * 2);

    // Vertex conflicts contribute a single point and metadata entry.
    for (const auto& v : vertex_collisions) {
        int a1 = std::get<0>(v);
        int a2 = std::get<1>(v);
        auto pos = std::get<2>(v);
        int t = std::get<3>(v);
        conflict_points.push_back(pos);
        conflict_meta.push_back(ConflictMeta{a1, a2, t, false, pos, {-1, -1}});
    }

    // Edge conflicts contribute both end points to improve spatial clustering.
    for (const auto& e : edge_collisions) {
        int a1 = std::get<0>(e);
        int a2 = std::get<1>(e);
        auto pos1 = std::get<2>(e);
        auto pos2 = std::get<3>(e);
        int t = std::get<4>(e);
        conflict_points.push_back(pos1);
        conflict_meta.push_back(ConflictMeta{a1, a2, t, true, pos1, pos2});
        conflict_points.push_back(pos2);
        conflict_meta.push_back(ConflictMeta{a1, a2, t, true, pos1, pos2});
    }
    return {conflict_points, conflict_meta};
}

//used locallywithout conflict points
std::vector<ConflictMeta> collect_conflicts_meta(
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& vertex_collisions,
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& edge_collisions) {
    std::vector<ConflictMeta> conflict_meta;
    conflict_meta.reserve(vertex_collisions.size() + edge_collisions.size());

    // Vertex conflicts contribute a single metadata entry.
    for (const auto& [agent1, agent2, pos, t] : vertex_collisions) {

        conflict_meta.push_back(ConflictMeta{agent1, agent2, t, false, pos, {-1, -1}});
    }

    // Edge conflicts contribute a single metadata entry.
    for (const auto& [agent1, agent2, pos1, pos2, t] : edge_collisions) {

        conflict_meta.push_back(ConflictMeta{agent1, agent2, t, true, pos1, pos2});
    }
    return conflict_meta;
}


// Helper: create a spatial conflict map for efficient conflict queries
// Returns a 2D array where conflict_map[r][c] = conflict_index if there's a conflict at (r,c), -1 otherwise
std::vector<std::vector<std::vector<int>>> create_conflict_map_2D(
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<std::vector<char>>& map) {
    int rows = map.size();
    int cols = map[0].size();
    std::vector<std::vector<std::vector<int>>> conflict_map(rows, std::vector<std::vector<int>>(cols));
    
    for (size_t i = 0; i < conflict_meta.size(); ++i) {
        ConflictMeta conflict = conflict_meta[i];
        if (!conflict.is_edge) {
            auto [r, c] = conflict.pos1;
            if (r >= 0 && r < rows && c >= 0 && c < cols) {
                conflict_map[r][c].push_back(static_cast<int>(i));
            } else {
                std::cerr << "[Create_conflict_map_2D] ERROR: conflict position out of bounds" << std::endl;
            }
        }
        if (conflict.is_edge) {
            auto [r1, c1] = conflict.pos1;
            if (r1 >= 0 && r1 < rows && c1 >= 0 && c1 < cols) {
                conflict_map[r1][c1].push_back(static_cast<int>(i));
            } else {
                std::cerr << "[Create_conflict_map_2D] ERROR: conflict position out of bounds" << std::endl;
            }
            auto [r2, c2] = conflict.pos2;
            if (r2 >= 0 && r2 < rows && c2 >= 0 && c2 < cols) {
                conflict_map[r2][c2].push_back(static_cast<int>(i));
            } else {
                std::cerr << "[Create_conflict_map_2D] ERROR: conflict position out of bounds" << std::endl;
            }
        }
    }

    return conflict_map;
}



void CurrentSolution::update_with_local_paths_and_pseudo_agents(
    const LocalZoneState& local_zone_state,
    const std::unordered_map<int, std::vector<std::pair<int,int>>>& solved_segment_paths,
    const std::vector<std::vector<char>>& map) {

    const LocalZoneValidationResult validation =
        validate_local_zone_state(local_zone_state, *this);
    if (!validation.valid) {
        std::cerr << "[Current_Solution] ERROR: Refusing invalid local-zone state: "
                  << validation.message << std::endl;
        return;
    }

    lnssat::debug_log() << "[LNS] Updating global solution with pseudo-agent local paths..." << std::endl;

    std::set<int> processed_segments;
    std::unordered_map<int, int> segment_to_agent;
    segment_to_agent.reserve(local_zone_state.segments.size());
    for (const auto& segment : local_zone_state.segments) {
        segment_to_agent[segment.segment_id] = segment.original_id;
    }

    for (const auto& [agent_id, segment_indices] : local_zone_state.original_to_segments) {
        lnssat::debug_log() << "[Current_Solution] Updating global solution for agent " << agent_id << std::endl;
        auto global_it = agent_paths.find(agent_id);
        if (global_it == agent_paths.end()) {
            std::cerr << "[ERROR] Missing global path for Agent " << agent_id
                      << " when applying pseudo-agent update" << std::endl;
            continue;
        }
        auto& global_path = global_it->second;
        if (global_path.empty()) {
            std::cerr << "[WARNING] Agent " << agent_id
                      << " has empty global path; skipping pseudo-agent update" << std::endl;
            continue;
        }
        
        // Ensure chronological processing of all segments for this agent.
        std::vector<size_t> ordered_indices = segment_indices;
        std::sort(ordered_indices.begin(), ordered_indices.end(), [&](size_t a, size_t b) {
            if (a >= local_zone_state.segments.size() || b >= local_zone_state.segments.size()) {
                return a < b;
            }
            return local_zone_state.segments[a].entry_t < local_zone_state.segments[b].entry_t;
        });

        int cumulative_shift = 0;
        const int path_length = static_cast<int>(global_path.size());

        for (size_t idx : ordered_indices) {
            if (idx >= local_zone_state.segments.size()) {
                std::cerr << "[Current_Solution] ERROR: Segment index " << idx
                          << " out of range for agent " << agent_id << std::endl;
                continue;
            }

            const LocalSegment& segment = local_zone_state.segments[idx];
            processed_segments.insert(segment.segment_id);

            int original_entry = segment.original_entry_t >= 0 ? segment.original_entry_t : segment.entry_t;
            int original_exit = segment.original_exit_t >= 0 ? segment.original_exit_t : segment.exit_t;
            if (original_entry > original_exit) {
                std::cerr << "[Current_Solution] ERROR: Segment " << segment.segment_id
                          << " has invalid original bounds [" << original_entry
                          << ", " << original_exit << "]" << std::endl;
                continue;
            }
            int new_entry = segment.entry_t;
            int new_exit = segment.exit_t;
            if (new_entry < 0 || new_exit < new_entry) {
                std::cerr << "[Current_Solution] ERROR: Segment " << segment.segment_id
                          << " has invalid new bounds [" << new_entry
                          << ", " << new_exit << "]" << std::endl;
                continue;
            }

            if (new_exit >= path_length) {
                std::cerr << "[Current_Solution] ERROR: Segment " << segment.segment_id
                          << " new exit timestep " << new_exit
                          << " exceeds global path bounds (size " << path_length
                          << ") for agent " << agent_id << std::endl;
                continue;
            }

            int shifted_entry = original_entry + cumulative_shift;
            int shifted_exit = original_exit + cumulative_shift;
            if (shifted_entry != new_entry) {
                std::cerr << "[Current_Solution] WARNING: Segment " << segment.segment_id
                          << " entry mismatch after cumulative shift (expected "
                          << shifted_entry << " got " << new_entry << ")" << std::endl;
                shifted_entry = new_entry;
            }
            if (shifted_exit != new_exit) {
                lnssat::debug_log() << "[Current_Solution] INFO: Segment " << segment.segment_id
                          << " exit adjusted from " << shifted_exit
                          << " to " << new_exit << " due to waiting time" << std::endl;
            }

            int delta = new_exit - shifted_exit;
            if (delta < 0) {
                std::cerr << "[Current_Solution] WARNING: Segment " << segment.segment_id
                          << " shortened by " << -delta
                          << " timesteps; shrinking not supported yet" << std::endl;
                continue;
            }
            if (delta > 0) {
                lnssat::debug_log() << "[Current_Solution] Segment " << segment.segment_id
                          << " lengthened by " << delta
                          << " timesteps; shifting suffix" << std::endl;
                //verify global solution for consistency
                if (!verify_path_consistency(global_path, map)) {
                    std::cerr << "[Current_Solution] ERROR: Global solution is not consistent before shifting suffix" << std::endl;
                }else{
                    lnssat::debug_log() << "[Current_Solution] Global solution is consistent before shifting suffix" << std::endl;
                }
                //we dont need this anymore because we are keeping the paths at correct lengths 
                /*for (int t = path_length - 1; t >= new_exit + 1; --t) {
                    int src = t - delta;
                    if (src >= shifted_exit + 1 && src < path_length) {
                        global_path[t] = global_path[src];
                    }
                }*/
                //verify global solution for consistency
                if (!verify_path_consistency(global_path, map)) {
                    std::cerr << "[Current_Solution] ERROR: Global solution is not consistent after shifting suffix" << std::endl;
                }else{
                    lnssat::debug_log() << "[Current_Solution] Global solution is consistent after shifting suffix" << std::endl;
                }
            }

            const std::vector<std::pair<int,int>>* local_path_ptr = &segment.path;
            auto solved_it = solved_segment_paths.find(segment.segment_id);
            if (solved_it != solved_segment_paths.end()) {
                local_path_ptr = &solved_it->second;
            }

            int expected_length = new_exit - new_entry + 1;
            if (static_cast<int>(local_path_ptr->size()) != expected_length) {
                std::cerr << "[Current_Solution] ERROR: Segment " << segment.segment_id
                          << " local path length (" << local_path_ptr->size()
                          << ") does not match expected length (" << expected_length
                          << ")" << std::endl;
                continue;
            }
            if (new_entry < 0 || new_entry + expected_length > path_length) {
                std::cerr << "[Current_Solution] ERROR: Segment " << segment.segment_id
                          << " replacement range [" << new_entry << ", " << new_exit
                          << "] exceeds path bounds for agent " << agent_id << std::endl;
                continue;
            }

            if (local_path_ptr->empty()) {
                std::cerr << "[Current_Solution] ERROR: Segment " << segment.segment_id
                          << " local path is empty for agent " << agent_id << std::endl;
                continue;
            }

            //update global path
            for (int i = 0; i < expected_length; ++i) {
                global_path[new_entry + i] = (*local_path_ptr)[i];
            }

            //verify global solution for consistency
            if (!verify_path_consistency(global_path, map)) {
                std::cerr << "[Current_Solution] ERROR: Global solution is not consistent after updating with local path" << std::endl;
            }else{
                lnssat::debug_log() << "[Current_Solution] Global solution is consistent after updating with local path" << std::endl;
            }

            cumulative_shift += delta;
        }

        lnssat::debug_log() << "[LNS] Successfully updated global solution with pseudo-agent local paths!" << std::endl;
    }

    //check if all segments were processed
    for (const auto& [segment_id, path] : solved_segment_paths) {
        (void)path;
        if (processed_segments.count(segment_id) > 0) continue;

        auto it = segment_to_agent.find(segment_id);
        if (it != segment_to_agent.end()) {
            std::cerr << "[Current_Solution] WARNING: Skipping segment " << segment_id
                      << " for Agent " << it->second
                      << " because no ordering information was available" << std::endl;
        } else {
            std::cerr << "[Current_Solution] WARNING: Skipping segment " << segment_id
                      << " (segment not present in LocalZoneState)" << std::endl;
        }
    }

    // Update the path map to reflect the new paths
    lnssat::debug_log() << "[LNS] Updating path map with new local paths..." << std::endl;
    create_path_map();

    lnssat::debug_log() << "[LNS] Successfully updated global solution with local paths and pseudo agents!" << std::endl;
}
