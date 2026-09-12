#include "lnssat/Create_Local_Problem.h"

#include <iostream>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using Position = std::pair<int, int>;
using Path = std::vector<Position>;

const std::vector<std::vector<char>> kGrid(2, std::vector<char>(6, '.'));
const std::vector<std::vector<std::vector<int>>> kNoConflictMap;
const std::vector<ConflictMeta> kNoConflicts;

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << std::endl;
    }
    return condition;
}

CurrentSolution make_solution(const Path& path) {
    CurrentSolution solution(
        2,
        6,
        static_cast<int>(path.size()) - 1,
        1,
        std::vector<Position>{path.front()},
        std::vector<Position>{path.back()});
    solution.agent_paths[0] = path;
    solution.agent_waiting_time[0] = 0;
    solution.create_path_map();
    return solution;
}

LocalZoneState build_state(
    CurrentSolution& solution,
    const std::set<Position>& zone,
    int start_t,
    int end_t,
    const std::unordered_map<int, std::vector<int>>& pseudo_ids = {}) {
    return build_local_problem_for_zone(
        solution,
        zone,
        kGrid,
        kGrid,
        kNoConflictMap,
        kNoConflicts,
        0,
        start_t,
        end_t,
        pseudo_ids);
}

void refresh(LocalZoneState& state,
             CurrentSolution& solution,
             const std::set<Position>& zone,
             int previous_end_t,
             int new_end_t) {
    state.zone_end_t = new_end_t;
    refresh_zone_after_extension(
        state,
        solution,
        zone,
        previous_end_t,
        kGrid,
        kGrid,
        kNoConflictMap,
        kNoConflicts,
        0);
}

bool test_time_extension_continues_segment() {
    const Path path{{0, 0}, {0, 1}, {0, 1}, {0, 2}};
    CurrentSolution solution = make_solution(path);
    const std::set<Position> zone{{0, 1}};
    LocalZoneState state = build_state(solution, zone, 0, 1);

    refresh(state, solution, zone, 1, 2);

    bool ok = true;
    ok &= expect(state.segments.size() == 1,
                 "time extension: a continuous visit should remain one segment");
    if (state.segments.size() == 1) {
        const LocalSegment& segment = state.segments[0];
        ok &= expect(segment.entry_t == 1 && segment.exit_t == 2,
                     "time extension: continuous segment has wrong bounds");
        ok &= expect(segment.path == Path({{0, 1}, {0, 1}}),
                     "time extension: continuous segment path was not extended");
        ok &= expect(segment.mdd && segment.mdd->levels.begin()->first == 1 &&
                         segment.mdd->levels.rbegin()->first == 2,
                     "time extension: MDD bounds were not refreshed");
    }
    return ok;
}

bool test_time_extension_adds_reentry_segment() {
    const Path path{{0, 0}, {0, 1}, {0, 2}, {0, 1}, {0, 0}};
    CurrentSolution solution = make_solution(path);
    const std::set<Position> zone{{0, 1}};
    LocalZoneState state = build_state(solution, zone, 0, 1);

    refresh(state, solution, zone, 1, 3);

    bool ok = true;
    ok &= expect(state.segments.size() == 2,
                 "time extension: separated re-entry must create a pseudo segment");
    ok &= expect(state.original_to_pseudo_ids.at(0) == std::vector<int>{1},
                 "time extension: separated re-entry did not receive pseudo ID 1");
    if (state.segments.size() == 2) {
        ok &= expect(state.segments[0].entry_t == 1 && state.segments[0].exit_t == 1,
                     "time extension: first visit was incorrectly enlarged");
        ok &= expect(state.segments[1].segment_id == 1 &&
                         state.segments[1].entry_t == 3 &&
                         state.segments[1].exit_t == 3,
                     "time extension: re-entry segment has wrong identity or bounds");
    }
    return ok;
}

bool test_spatial_expansion_gains_segment() {
    const Path path{{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}};
    CurrentSolution solution = make_solution(path);
    LocalZoneState narrow = build_state(solution, {{0, 1}}, 0, 4);
    LocalZoneState expanded = build_state(
        solution, {{0, 1}, {0, 3}}, 0, 4, narrow.original_to_pseudo_ids);

    bool ok = true;
    ok &= expect(expanded.segments.size() == 2,
                 "spatial expansion: expected a newly covered second segment");
    ok &= expect(expanded.original_to_pseudo_ids.at(0) == std::vector<int>{1},
                 "spatial expansion: new segment did not receive pseudo ID 1");
    if (expanded.segments.size() == 2) {
        ok &= expect(expanded.segments[0].segment_id == 0 &&
                         expanded.segments[1].segment_id == 1,
                     "spatial expansion: real and pseudo identities are unstable");
    }
    return ok;
}

bool test_spatial_expansion_merges_segments() {
    const Path path{{0, 0}, {0, 1}, {0, 2}, {0, 1}, {0, 0}};
    CurrentSolution solution = make_solution(path);
    LocalZoneState narrow = build_state(solution, {{0, 1}}, 0, 4);
    LocalZoneState expanded = build_state(
        solution, {{0, 1}, {0, 2}}, 0, 4, narrow.original_to_pseudo_ids);

    bool ok = true;
    ok &= expect(narrow.segments.size() == 2,
                 "spatial merge setup: narrow zone should have two visits");
    ok &= expect(expanded.segments.size() == 1,
                 "spatial expansion: adjacent visits should merge into one segment");
    ok &= expect(expanded.original_to_pseudo_ids.at(0).empty(),
                 "spatial expansion: merged-away pseudo ID remained active");
    ok &= expect(expanded.segment_index_by_id.count(1) == 0,
                 "spatial expansion: merged-away pseudo ID remained indexed");
    return ok;
}

bool test_reassembly_preserves_outside_ranges() {
    const Path original{{0, 0},
                        {0, 1}, {0, 1}, {0, 2}, {0, 2}, {0, 3},
                        {0, 4},
                        {0, 3}, {0, 3}, {0, 2}, {0, 2}, {0, 1},
                        {0, 0}};
    const Path first_repair{{0, 1}, {1, 1}, {1, 2}, {1, 3}, {0, 3}};
    const Path second_repair{{0, 3}, {1, 3}, {1, 2}, {1, 1}, {0, 1}};
    CurrentSolution solution = make_solution(original);
    const std::set<Position> zone{{0, 1}, {0, 2}, {0, 3}};
    LocalZoneState state = build_state(solution, zone, 0, 12);

    bool ok = true;
    ok &= expect(state.segments.size() == 2,
                 "reassembly setup: expected two authorized segments");
    if (state.segments.size() != 2) {
        return false;
    }

    solution.update_with_local_paths_and_pseudo_agents(
        state,
        {{state.segments[0].segment_id, first_repair},
         {state.segments[1].segment_id, second_repair}},
        kGrid);

    const Path& updated = solution.agent_paths.at(0);
    for (int outside_t : {0, 6, 12}) {
        ok &= expect(updated[outside_t] == original[outside_t],
                     "reassembly: path changed outside an authorized segment at t=" +
                         std::to_string(outside_t));
    }
    ok &= expect(Path(updated.begin() + 1, updated.begin() + 6) == first_repair,
                 "reassembly: first pseudo-agent repair was not spliced exactly");
    ok &= expect(Path(updated.begin() + 7, updated.begin() + 12) == second_repair,
                 "reassembly: second pseudo-agent repair was not spliced exactly");
    return ok;
}

}  // namespace

int main() {
    bool ok = true;
    ok &= test_time_extension_continues_segment();
    ok &= test_time_extension_adds_reentry_segment();
    ok &= test_spatial_expansion_gains_segment();
    ok &= test_spatial_expansion_merges_segments();
    ok &= test_reassembly_preserves_outside_ranges();

    if (!ok) {
        return 1;
    }
    std::cout << "PASS: pseudo-agent refresh, expansion, and reassembly" << std::endl;
    return 0;
}
