#include "../Create_Local_Problem.h"
#include "../Waiting_time_Solve.h"

#include <iostream>
#include <random>
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

LocalZoneState build_state(CurrentSolution& solution,
                           const std::set<Position>& zone) {
    return build_local_problem_for_zone(
        solution,
        zone,
        kGrid,
        kGrid,
        kNoConflictMap,
        kNoConflicts,
        0,
        0,
        solution.max_timestep);
}

bool expect_invalid(const LocalZoneState& state,
                    const CurrentSolution& solution,
                    const std::string& diagnostic_fragment) {
    const LocalZoneValidationResult result =
        validate_local_zone_state(state, solution);
    return expect(!result.valid,
                  "validator accepted malformed state: " + diagnostic_fragment) &&
           expect(result.message.find(diagnostic_fragment) != std::string::npos,
                  "validator diagnostic did not mention " + diagnostic_fragment +
                      "; got: " + result.message);
}

bool test_cross_index_and_interval_invariants() {
    const Path path{{0, 0}, {0, 1}, {0, 2}, {0, 1}, {0, 0}};
    CurrentSolution solution = make_solution(path);
    const LocalZoneState valid = build_state(solution, {{0, 1}});

    bool ok = true;
    ok &= expect(validate_local_zone_state(valid, solution).valid,
                 "validator rejected a valid two-segment state");

    LocalZoneState duplicate_id = valid;
    duplicate_id.segments[1].segment_id = 0;
    ok &= expect_invalid(duplicate_id, solution, "unique");

    LocalZoneState bad_index = valid;
    bad_index.segment_index_by_id[1] = 0;
    ok &= expect_invalid(bad_index, solution, "segment_index_by_id");

    LocalZoneState bad_order = valid;
    bad_order.original_to_segments[0] = {1, 0};
    ok &= expect_invalid(bad_order, solution, "first segment");

    LocalZoneState missing_index = valid;
    missing_index.original_to_segments[0] = {0};
    ok &= expect_invalid(missing_index, solution, "missing from original_to_segments");

    LocalZoneState overlapping = valid;
    overlapping.segments[1].entry_t = 1;
    overlapping.segments[1].exit_t = 1;
    overlapping.segments[1].original_entry_t = 1;
    overlapping.segments[1].original_exit_t = 1;
    overlapping.segments[1].path = {{0, 1}};
    overlapping.segments[1].mdd = valid.segments[0].mdd;
    ok &= expect_invalid(overlapping, solution, "ordered and non-overlapping");

    LocalZoneState bad_mapping = valid;
    bad_mapping.original_to_pseudo_ids[0].clear();
    ok &= expect_invalid(bad_mapping, solution, "pseudo-ID mapping");

    LocalZoneState bad_owner = valid;
    bad_owner.segments[1].original_id = 1;
    ok &= expect_invalid(bad_owner, solution, "ownership");

    LocalZoneState bad_next_id = valid;
    bad_next_id.next_pseudo_id = 1;
    ok &= expect_invalid(bad_next_id, solution, "next_pseudo_id");
    return ok;
}

bool test_path_and_mdd_invariants() {
    const Path path{{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}};
    CurrentSolution solution = make_solution(path);
    const LocalZoneState valid = build_state(solution, {{0, 1}, {0, 2}, {0, 3}});

    bool ok = true;
    ok &= expect(validate_local_zone_state(valid, solution).valid,
                 "validator rejected a valid multi-timestep segment");

    LocalZoneState bad_length = valid;
    bad_length.segments[0].path.pop_back();
    ok &= expect_invalid(bad_length, solution, "path length");

    LocalZoneState bad_continuity = valid;
    bad_continuity.segments[0].path[1] = {1, 5};
    ok &= expect_invalid(bad_continuity, solution, "continuous");

    LocalZoneState bad_mdd = valid;
    bad_mdd.segments[0].mdd = valid.segments[0].mdd->copy();
    bad_mdd.segments[0].mdd->levels.erase(bad_mdd.segments[0].entry_t);
    ok &= expect_invalid(bad_mdd, solution, "MDD time bounds");

    LocalZoneState bad_endpoint = valid;
    bad_endpoint.segments[0].mdd = valid.segments[0].mdd->copy();
    bad_endpoint.segments[0].mdd->levels.begin()->second.front()->position = {0, 0};
    ok &= expect_invalid(bad_endpoint, solution, "endpoints");
    return ok;
}

bool test_invalid_state_is_not_integrated() {
    const Path path{{0, 0}, {0, 1}, {0, 2}, {0, 1}, {0, 0}};
    CurrentSolution solution = make_solution(path);
    LocalZoneState invalid = build_state(solution, {{0, 1}});
    invalid.segment_index_by_id[1] = 0;
    const auto paths_before = solution.agent_paths;

    solution.update_with_local_paths_and_pseudo_agents(
        invalid, {{0, {{0, 1}}}, {1, {{0, 1}}}}, kGrid);
    return expect(solution.agent_paths == paths_before,
                  "invalid local-zone state changed the global solution");
}

bool test_invalid_construction_returns_invalid_state() {
    const Path discontinuous_path{{0, 0}, {0, 2}};
    CurrentSolution solution = make_solution(discontinuous_path);
    std::mt19937 rng(7);
    const WaitingSolveResult result = lazy_solve_with_waiting_time(
        solution,
        kGrid,
        kGrid,
        {{0, 0}, {0, 1}, {0, 2}},
        kNoConflicts,
        {},
        kNoConflictMap,
        0,
        1,
        0,
        0,
        rng);

    return expect(result.status == SolveStatus::InvalidState,
                  "invalid constructed state did not propagate InvalidState") &&
           expect(result.message.find("after construction") != std::string::npos,
                  "InvalidState diagnostic did not identify construction boundary");
}

}  // namespace

int main() {
    bool ok = true;
    ok &= test_cross_index_and_interval_invariants();
    ok &= test_path_and_mdd_invariants();
    ok &= test_invalid_state_is_not_integrated();
    ok &= test_invalid_construction_returns_invalid_state();

    if (!ok) {
        return 1;
    }
    std::cout << "PASS: LocalZoneState invariants and InvalidState propagation"
              << std::endl;
    return 0;
}
