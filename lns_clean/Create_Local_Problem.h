#pragma once

#include "Current_Solution.h"
#include "MDD.h"

#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// Forward declarations
class MDDConstructor;

// Stores a single contiguous visit of an agent (or pseudo agent) inside the
// local zone. Each segment has a stable identifier that can be used by the CNF
// solver. "original_id" refers back to the real agent that owns the segment.
struct LocalSegment {
    int segment_id = -1;
    int original_id = -1;
    int entry_t = -1;
    int exit_t = -1;
    int original_entry_t = -1;
    int original_exit_t = -1;
    std::vector<std::pair<int,int>> path;
    std::shared_ptr<MDD> mdd;
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> edge_collisions;
};

// Aggregate state for the local zone problem. Tracks all segments, their
// ordering per original agent and helper indices for fast lookups.
struct LocalZoneState {
    std::vector<LocalSegment> segments;
    std::unordered_map<int, std::vector<size_t>> original_to_segments;
    std::unordered_map<int, size_t> segment_index_by_id;
    std::unordered_map<int, std::vector<int>> original_to_pseudo_ids;
    int next_pseudo_id = 0;
    int zone_start_t = 0;
    int zone_end_t = 0;
};

void align_mdd_to_time_window(std::shared_ptr<MDD> mdd,
    int entry_t, int exit_t,
    int start_t, int end_t);

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
    const std::unordered_map<int, std::vector<int>>& agent_to_pseudo_agent_id = {});

std::unordered_map<int, std::shared_ptr<MDD>> build_segment_mdd_map(const LocalZoneState& state);
std::unordered_map<int, std::pair<int,int>> build_segment_entry_exit_map(const LocalZoneState& state);
std::unordered_map<int, std::vector<std::pair<int,int>>> build_segment_path_map(const LocalZoneState& state);
