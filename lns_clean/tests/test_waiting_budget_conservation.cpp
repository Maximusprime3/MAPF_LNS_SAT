#include "../Waiting_time_Solve.h"

#include <algorithm>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {

using Position = std::pair<int, int>;
using Path = std::vector<Position>;
using Grid = std::vector<std::vector<char>>;

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << std::endl;
    }
    return condition;
}

CurrentSolution make_solution(
    const Grid& grid,
    const std::vector<Position>& starts,
    const std::vector<Position>& goals,
    const std::vector<Path>& paths) {
    const int makespan = static_cast<int>(paths.front().size()) - 1;
    CurrentSolution solution(
        static_cast<int>(grid.size()),
        static_cast<int>(grid.front().size()),
        makespan,
        static_cast<int>(paths.size()),
        starts,
        goals);
    for (size_t agent_id = 0; agent_id < paths.size(); ++agent_id) {
        solution.agent_paths[static_cast<int>(agent_id)] = paths[agent_id];
    }
    solution.calculate_waiting_times(goals, makespan);
    solution.create_path_map();
    return solution;
}

std::vector<std::vector<std::vector<int>>> empty_conflict_map(
    const Grid& grid) {
    return std::vector<std::vector<std::vector<int>>>(
        grid.size(),
        std::vector<std::vector<int>>(grid.front().size()));
}

WaitingSolveResult solve_zone(
    CurrentSolution& solution,
    const Grid& grid,
    const std::set<Position>& zone,
    int end_t,
    int seed = 7) {
    std::mt19937 rng(seed);
    return lazy_solve_with_waiting_time(
        solution,
        grid,
        grid,
        zone,
        {},
        {},
        empty_conflict_map(grid),
        0,
        end_t,
        0,
        0,
        rng);
}

int total_waiting_budget(const CurrentSolution& solution) {
    int total = 0;
    for (const auto& [agent_id, waiting] : solution.agent_waiting_time) {
        (void)agent_id;
        total += waiting;
    }
    return total;
}

bool expect_fixed_horizon(const CurrentSolution& solution,
                          int expected_makespan,
                          const std::string& case_name) {
    bool ok = expect(solution.max_timestep == expected_makespan,
                     case_name + ": common makespan changed");
    for (const auto& [agent_id, path] : solution.agent_paths) {
        ok &= expect(
            static_cast<int>(path.size()) == expected_makespan + 1,
            case_name + ": agent " + std::to_string(agent_id) +
                " path length changed");
    }
    return ok;
}

bool expect_valid_goal_wait_suffixes(const CurrentSolution& solution,
                                     const std::string& case_name) {
    bool ok = true;
    for (const auto& [agent_id, path] : solution.agent_paths) {
        const int waiting = solution.get_waiting_time(agent_id);
        ok &= expect(waiting >= 0 &&
                         static_cast<size_t>(waiting) < path.size(),
                     case_name + ": invalid waiting budget for agent " +
                         std::to_string(agent_id));
        if (waiting < 0 || static_cast<size_t>(waiting) >= path.size()) {
            continue;
        }
        const size_t suffix_start =
            path.size() - static_cast<size_t>(waiting) - 1;
        for (size_t index = suffix_start; index < path.size(); ++index) {
            ok &= expect(
                path[index] == solution.goals[agent_id],
                case_name + ": agent " + std::to_string(agent_id) +
                    " lost its goal-wait suffix");
        }
    }
    return ok;
}

bool test_success_consumes_horizon_slack_once() {
    const Grid grid(3, std::vector<char>(3, '.'));
    const std::vector<Position> starts{{1, 0}, {0, 1}, {2, 2}};
    const std::vector<Position> goals{{1, 2}, {2, 1}, {2, 2}};
    const std::vector<Path> paths{
        {{1, 0}, {1, 1}, {1, 2}, {1, 2}},
        {{0, 1}, {1, 1}, {2, 1}, {2, 1}},
        {{2, 2}, {2, 2}, {2, 2}, {2, 2}}};
    CurrentSolution solution = make_solution(grid, starts, goals, paths);
    const auto budgets_before = solution.agent_waiting_time;
    const Path unaffected_before = solution.agent_paths.at(2);
    const int total_before = total_waiting_budget(solution);

    const WaitingSolveResult result = solve_zone(
        solution,
        grid,
        {{1, 0}, {1, 1}, {1, 2}, {0, 1}, {2, 1}},
        2);

    bool ok = true;
    ok &= expect(result.solved(),
                 "successful repair: real slack/SAT orchestration did not solve");
    ok &= expect(result.waiting_attempts.size() == 2,
                 "successful repair: expected one failed SAT attempt and one retry");
    const int applied_attempts = static_cast<int>(std::count_if(
        result.waiting_attempts.begin(),
        result.waiting_attempts.end(),
        [](const WaitingAttemptMetrics& metrics) {
            return metrics.applied_waiting_time;
        }));
    ok &= expect(applied_attempts == 1,
                 "successful repair: slack was not applied exactly once");
    ok &= expect(total_waiting_budget(solution) == total_before - 2,
                 "successful repair: absorbed horizon slack was not charged once");
    const int consumed_by_conflicting_agents =
        (budgets_before.at(0) - solution.get_waiting_time(0)) +
        (budgets_before.at(1) - solution.get_waiting_time(1));
    ok &= expect(consumed_by_conflicting_agents == 2,
                 "successful repair: conflicting agents consumed the wrong slack");
    ok &= expect(solution.get_waiting_time(2) == budgets_before.at(2),
                 "successful repair: unaffected agent budget changed");
    ok &= expect(solution.agent_paths.at(2) == unaffected_before,
                 "successful repair: unaffected agent path changed");
    ok &= expect_fixed_horizon(solution, 3, "successful repair");
    ok &= expect_valid_goal_wait_suffixes(solution, "successful repair");
    return ok;
}

bool test_unsat_retries_roll_back_all_state() {
    const Grid grid(1, std::vector<char>(3, '.'));
    const std::vector<Position> starts{{0, 0}, {0, 2}};
    const std::vector<Position> goals{{0, 2}, {0, 0}};
    const std::vector<Path> paths{
        {{0, 0}, {0, 1}, {0, 2}, {0, 2}, {0, 2}},
        {{0, 2}, {0, 1}, {0, 0}, {0, 0}, {0, 0}}};
    CurrentSolution solution = make_solution(grid, starts, goals, paths);
    const auto paths_before = solution.agent_paths;
    const auto budgets_before = solution.agent_waiting_time;

    const WaitingSolveResult result =
        solve_zone(solution, grid, {{0, 0}, {0, 1}, {0, 2}}, 2);

    bool ok = true;
    ok &= expect(result.status == SolveStatus::Exhausted,
                 "UNSAT retries: corridor instance was expected to exhaust");
    ok &= expect(result.waiting_attempts.size() >= 2,
                 "UNSAT retries: multiple real SAT attempts were not exercised");
    ok &= expect(std::any_of(
                     result.waiting_attempts.begin(),
                     result.waiting_attempts.end(),
                     [](const WaitingAttemptMetrics& metrics) {
                         return metrics.applied_waiting_time;
                     }),
                 "UNSAT retries: no speculative slack mutation occurred");
    ok &= expect(solution.agent_paths == paths_before,
                 "UNSAT retries: speculative paths were not rolled back");
    ok &= expect(solution.agent_waiting_time == budgets_before,
                 "UNSAT retries: waiting budgets were not rolled back");
    ok &= expect_fixed_horizon(solution, 4, "UNSAT retries");
    ok &= expect_valid_goal_wait_suffixes(solution, "UNSAT retries");
    return ok;
}

bool test_early_validation_failure_rolls_back() {
    const Grid grid(2, std::vector<char>(3, '.'));
    const std::vector<Position> starts{{0, 0}, {1, 0}};
    const std::vector<Position> goals{{0, 2}, {1, 2}};
    const std::vector<Path> paths{
        {{0, 0}, {0, 2}, {0, 2}},
        {{1, 0}, {1, 1}, {1, 2}}};
    CurrentSolution solution = make_solution(grid, starts, goals, paths);
    const auto paths_before = solution.agent_paths;
    const auto budgets_before = solution.agent_waiting_time;

    const WaitingSolveResult result =
        solve_zone(solution, grid, {{0, 0}, {0, 1}, {0, 2}}, 2);

    bool ok = true;
    ok &= expect(result.status == SolveStatus::InvalidState,
                 "early validation: malformed segment was not rejected");
    ok &= expect(result.message.find("after construction") != std::string::npos,
                 "early validation: diagnostic missed the construction boundary");
    ok &= expect(solution.agent_paths == paths_before,
                 "early validation: paths changed before rollback");
    ok &= expect(solution.agent_waiting_time == budgets_before,
                 "early validation: budgets changed before rollback");
    ok &= expect_fixed_horizon(solution, 2, "early validation");
    ok &= expect_valid_goal_wait_suffixes(solution, "early validation");
    return ok;
}

bool test_non_goal_segment_charges_shifted_suffix_once() {
    const Grid grid(5, std::vector<char>(5, '.'));
    const std::vector<Position> starts{{2, 0}, {0, 2}, {4, 4}};
    const std::vector<Position> goals{{2, 4}, {4, 2}, {4, 4}};
    const std::vector<Path> paths{
        {{2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4},
         {2, 4}, {2, 4}, {2, 4}, {2, 4}},
        {{0, 2}, {1, 2}, {2, 2}, {3, 2}, {4, 2},
         {4, 2}, {4, 2}, {4, 2}, {4, 2}},
        {{4, 4}, {4, 4}, {4, 4}, {4, 4}, {4, 4},
         {4, 4}, {4, 4}, {4, 4}, {4, 4}}};
    CurrentSolution solution = make_solution(grid, starts, goals, paths);
    const auto budgets_before = solution.agent_waiting_time;
    const Path unaffected_before = solution.agent_paths.at(2);
    const int total_before = total_waiting_budget(solution);

    const WaitingSolveResult result = solve_zone(
        solution,
        grid,
        {{2, 1}, {2, 2}, {2, 3}, {1, 2}, {3, 2}},
        2,
        19);

    bool ok = true;
    ok &= expect(result.solved(),
                 "non-goal segment: real orchestration did not solve");
    ok &= expect(result.waiting_attempts.size() == 2,
                 "non-goal segment: expected one failed attempt and one retry");
    ok &= expect(total_waiting_budget(solution) == total_before - 1,
                 "non-goal segment: shifted suffix was not charged once");
    ok &= expect(solution.get_waiting_time(2) == budgets_before.at(2),
                 "non-goal segment: unaffected agent budget changed");
    ok &= expect(solution.agent_paths.at(2) == unaffected_before,
                 "non-goal segment: unaffected agent path changed");
    ok &= expect_fixed_horizon(solution, 8, "non-goal segment");
    ok &= expect_valid_goal_wait_suffixes(solution, "non-goal segment");
    return ok;
}

bool test_multi_retry_success_conserves_horizon_budgets() {
    const Grid grid(5, std::vector<char>(5, '.'));
    const std::vector<Position> starts{
        {2, 0}, {2, 4}, {0, 2}, {4, 2}};
    const std::vector<Position> goals{
        {2, 4}, {2, 0}, {4, 2}, {0, 2}};
    const std::vector<Path> paths{
        {{2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4},
         {2, 4}, {2, 4}, {2, 4}, {2, 4}},
        {{2, 4}, {2, 3}, {2, 2}, {2, 1}, {2, 0},
         {2, 0}, {2, 0}, {2, 0}, {2, 0}},
        {{0, 2}, {1, 2}, {2, 2}, {3, 2}, {4, 2},
         {4, 2}, {4, 2}, {4, 2}, {4, 2}},
        {{4, 2}, {3, 2}, {2, 2}, {1, 2}, {0, 2},
         {0, 2}, {0, 2}, {0, 2}, {0, 2}}};
    CurrentSolution solution = make_solution(grid, starts, goals, paths);

    std::set<Position> zone;
    for (int index = 0; index < 5; ++index) {
        zone.insert({2, index});
        zone.insert({index, 2});
    }
    const WaitingSolveResult result =
        solve_zone(solution, grid, zone, 4, 11);

    bool ok = true;
    ok &= expect(result.solved(),
                 "multi-retry success: real orchestration did not solve");
    ok &= expect(result.waiting_attempts.size() >= 2,
                 "multi-retry success: retry path was not exercised");
    for (const auto& [agent_id, path] : solution.agent_paths) {
        (void)path;
        ok &= expect(
            solution.get_waiting_time(agent_id) == 1,
            "multi-retry success: horizon slack was not conserved for agent " +
                std::to_string(agent_id));
    }
    ok &= expect_fixed_horizon(solution, 8, "multi-retry success");
    ok &= expect_valid_goal_wait_suffixes(solution, "multi-retry success");
    return ok;
}

}  // namespace

int main() {
    bool ok = true;
    ok &= test_success_consumes_horizon_slack_once();
    ok &= test_unsat_retries_roll_back_all_state();
    ok &= test_early_validation_failure_rolls_back();
    ok &= test_non_goal_segment_charges_shifted_suffix_once();
    ok &= test_multi_retry_success_conserves_horizon_budgets();

    if (!ok) {
        return 1;
    }
    std::cout << "PASS: waiting-budget conservation across commit and rollback"
              << std::endl;
    return 0;
}
