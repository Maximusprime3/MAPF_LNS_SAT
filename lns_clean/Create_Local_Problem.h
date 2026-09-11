#pragma once

#include "Current_Solution.h"
#include "Deadline.h"
#include "../mdd/MDD.h"

#include <memory>
#include <set>
#include <string>
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
    SolverDeadline deadline;
    std::vector<LocalSegment> segments;
    std::unordered_map<int, std::vector<size_t>> original_to_segments; //original_id -> list of segment indices
    std::unordered_map<int, size_t> segment_index_by_id; //segment_id (pseudo_agent_id)-> segment index
    std::unordered_map<int, std::vector<int>> original_to_pseudo_ids; //original_id -> list of pseudo agent ids
    int next_pseudo_id = 0;
    int zone_start_t = 0;
    int zone_end_t = 0;
};

struct LocalZoneValidationResult {
    bool valid = false;
    std::string message;
};

// Checks the complete cross-indexed pseudo-agent representation without
// mutating it. Callers use the diagnostic to reject malformed local state
// before it reaches CNF construction or global path integration.
LocalZoneValidationResult validate_local_zone_state(
    const LocalZoneState& state,
    const CurrentSolution& current_solution);

void align_mdd_to_time_window(std::shared_ptr<MDD> mdd,
    int entry_t, int exit_t,
    int start_t, int end_t);

// Builds an MDD for a segment, adding a waiting tail at the global goal if present in the segment.
std::shared_ptr<MDD> build_segment_mdd_with_optional_wait_tail(
    const std::vector<std::vector<char>>& masked_map,
    const std::vector<std::pair<int,int>>& segment_path,
    const std::pair<int,int>& global_goal_pos,
    int segment_entry_t,
    int segment_exit_t,
    int window_start_t,
    int window_end_t,
    int agent_id,
    int forced_pre_tail_idx = -1);

// Builds an MDD for a segment while accounting for waiting time usage when the
// global goal lies on the segment path.
// The returned MDD always targets the segment's final position and only deducts
// waiting time if the agent's final goal-staying suffix lasts through the
// segment exit and the rest of its global path.
std::shared_ptr<MDD> build_segment_mdd(
    CurrentSolution& current_solution,
    const LocalSegment& segment,
    const std::vector<std::vector<char>>& masked_map,
    int window_start_t,
    int window_end_t,
    SolverDeadline deadline = {});

LocalZoneState build_local_problem_for_zone(
    CurrentSolution& current_solution,
    const std::set<std::pair<int,int>>& zone_positions_set,
    const std::vector<std::vector<char>>& masked_map,
    const std::vector<std::vector<char>>& grid,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<ConflictMeta>& conflict_meta,
    int offset,
    int start_t,
    int end_t,
    const std::unordered_map<int, std::vector<int>>& agent_to_pseudo_agent_id = {},
    SolverDeadline deadline = {});

std::unordered_map<int, std::shared_ptr<MDD>> build_segment_mdd_map(const LocalZoneState& state);
std::unordered_map<int, std::pair<int,int>> build_segment_entry_exit_time_map(const LocalZoneState& state);
std::unordered_map<int, std::vector<std::pair<int,int>>> build_segment_path_map(const LocalZoneState& state);


void refresh_zone_after_extension(
    LocalZoneState& state,
    CurrentSolution& current_solution,
    const std::set<std::pair<int,int>>& local_zone_positions,
    int previous_zone_end_t,
    const std::vector<std::vector<char>>& masked_map,
    const std::vector<std::vector<char>>& grid,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    const std::vector<ConflictMeta>& conflict_meta,
    int offset);
