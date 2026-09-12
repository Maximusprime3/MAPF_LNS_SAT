#include "lnssat/Create_Local_Problem.h"

#include <iostream>
#include <set>
#include <string>
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

LocalZoneState build_state(const Path& path,
                           const std::set<Position>& zone,
                           int start_t = 0,
                           int end_t = -1) {
    if (end_t < 0) {
        end_t = static_cast<int>(path.size()) - 1;
    }
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

    return build_local_problem_for_zone(
        solution,
        zone,
        kGrid,
        kGrid,
        kNoConflictMap,
        kNoConflicts,
        0,
        start_t,
        end_t);
}

bool expect_segment(const LocalSegment& segment,
                    int segment_id,
                    int entry_t,
                    int exit_t,
                    const Path& path,
                    const std::string& case_name) {
    bool ok = true;
    ok &= expect(segment.segment_id == segment_id,
                 case_name + ": unexpected segment ID");
    ok &= expect(segment.original_id == 0,
                 case_name + ": segment lost its real-agent owner");
    ok &= expect(segment.entry_t == entry_t && segment.exit_t == exit_t,
                 case_name + ": unexpected segment interval");
    ok &= expect(segment.original_entry_t == entry_t &&
                     segment.original_exit_t == exit_t,
                 case_name + ": original interval was not preserved");
    ok &= expect(segment.path == path,
                 case_name + ": segment path differs from the global path slice");
    ok &= expect(segment.mdd != nullptr && !segment.mdd->levels.empty(),
                 case_name + ": segment MDD was not constructed");
    if (segment.mdd && !segment.mdd->levels.empty()) {
        ok &= expect(segment.mdd->levels.begin()->first == entry_t &&
                         segment.mdd->levels.rbegin()->first == exit_t,
                     case_name + ": MDD time bounds do not match the segment");
    }
    return ok;
}

bool test_single_visit() {
    const Path path{{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}};
    const LocalZoneState state = build_state(path, {{0, 1}, {0, 2}});

    bool ok = true;
    ok &= expect(state.segments.size() == 1, "single visit: expected one segment");
    ok &= expect(state.original_to_segments.at(0) == std::vector<size_t>{0},
                 "single visit: missing chronological segment index");
    ok &= expect(state.original_to_pseudo_ids.at(0).empty(),
                 "single visit: unexpectedly allocated a pseudo ID");
    if (state.segments.size() == 1) {
        ok &= expect_segment(
            state.segments[0], 0, 1, 2, {{0, 1}, {0, 2}}, "single visit");
    }
    return ok;
}

bool test_reentry() {
    const Path path{{0, 0}, {0, 1}, {0, 2}, {0, 1}, {0, 0}};
    const LocalZoneState state = build_state(path, {{0, 1}});

    bool ok = true;
    ok &= expect(state.segments.size() == 2, "re-entry: expected two segments");
    ok &= expect(state.original_to_segments.at(0) == std::vector<size_t>({0, 1}),
                 "re-entry: segment order is not chronological");
    ok &= expect(state.original_to_pseudo_ids.at(0) == std::vector<int>{1},
                 "re-entry: expected one stable pseudo ID");
    ok &= expect(state.segment_index_by_id.at(0) == 0 &&
                     state.segment_index_by_id.at(1) == 1,
                 "re-entry: segment ID index is inconsistent");
    if (state.segments.size() == 2) {
        ok &= expect_segment(state.segments[0], 0, 1, 1, {{0, 1}}, "re-entry first");
        ok &= expect_segment(state.segments[1], 1, 3, 3, {{0, 1}}, "re-entry second");
    }
    return ok;
}

bool test_starts_inside_zone() {
    const Path path{{0, 0}, {0, 1}, {0, 2}, {0, 3}};
    const LocalZoneState state = build_state(path, {{0, 0}, {0, 1}});
    return expect(state.segments.size() == 1,
                  "starts inside: expected one segment") &&
           expect_segment(
               state.segments[0], 0, 0, 1, {{0, 0}, {0, 1}}, "starts inside");
}

bool test_ends_inside_zone() {
    const Path path{{0, 0}, {0, 1}, {0, 2}, {0, 3}};
    const LocalZoneState state = build_state(path, {{0, 2}, {0, 3}});
    return expect(state.segments.size() == 1,
                  "ends inside: expected one segment") &&
           expect_segment(
               state.segments[0], 0, 2, 3, {{0, 2}, {0, 3}}, "ends inside");
}

bool test_reaches_goal_inside_zone() {
    const Path path{{0, 0}, {0, 1}, {0, 2}, {0, 2}, {0, 2}};
    const LocalZoneState state = build_state(path, {{0, 1}, {0, 2}});
    return expect(state.segments.size() == 1,
                  "goal inside: expected one segment") &&
           expect_segment(state.segments[0],
                          0,
                          1,
                          4,
                          {{0, 1}, {0, 2}, {0, 2}, {0, 2}},
                          "goal inside");
}

}  // namespace

int main() {
    bool ok = true;
    ok &= test_single_visit();
    ok &= test_reentry();
    ok &= test_starts_inside_zone();
    ok &= test_ends_inside_zone();
    ok &= test_reaches_goal_inside_zone();

    if (!ok) {
        return 1;
    }
    std::cout << "PASS: deterministic pseudo-agent segmentation cases" << std::endl;
    return 0;
}
