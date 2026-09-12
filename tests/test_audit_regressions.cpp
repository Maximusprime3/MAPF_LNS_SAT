#include "lnssat/LNS.h"
#include "lnssat/Load_LNSProblem.h"
#include "lnssat/Local_Zone_Builder.h"
#include "lnssat/Create_Local_Problem.h"
#include "lnssat/Waiting_time_Solve.h"
#include "lnssat/Lazy_SAT_Solve.h"
#include "lnssat/SolutionVerifier.h"
#include "lnssat/mdd/MDDConstructor.h"
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
using Position = std::pair<int, int>;
using Path = std::vector<Position>;
using Grid = std::vector<std::vector<char>>;
constexpr const char* map_file = "audit-regression.map";
constexpr const char* scen_file = "audit-regression.scen";
bool expect(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}
void write_inputs(int count) {
    std::ofstream(map_file) << "type octile\nheight 1\nwidth 3\nmap\n...\n";
    std::ofstream scenario(scen_file);
    scenario << "version 1\n";
    for (int i = 0; i < count; ++i)
        scenario << "0\taudit-regression.map\t3\t1\t0\t0\t2\t0\t2\n";
}
bool input_contract() {
    bool ok = true;
    SolverConfig config;
    config.log_level = LogLevel::Quiet;
    write_inputs(1);
    auto result = LNS({map_file, scen_file, 2, 0}, config);
    ok &= expect(result.status == SolveStatus::InvalidInput && result.paths.empty(),
                 "two-agent request must not succeed with one path");
    ok &= expect(!load_problem(map_file, scen_file, 0, 0), "zero agents must not divide by zero");
    result = LNS({map_file, scen_file, 1, 0}, config);
    ok &= expect(result.solved() && result.paths.size() == 1, "complete request must still solve");
    write_inputs(3);
    ok &= expect(load_problem(map_file, scen_file, 2, 0).has_value(), "complete first group must load");
    ok &= expect(!load_problem(map_file, scen_file, 2, 1), "incomplete final group must not load");
    for (const char* bad_row : {"malformed row\n", "0\taudit-regression.map\t3\t1\t0x\t0\t2\t0\t2\n"}) {
        std::ofstream(scen_file) << "version 1\n" << bad_row
            << "0\taudit-regression.map\t3\t1\t0\t0\t2\t0\t2\n";
        ok &= expect(!load_problem(map_file, scen_file, 1, 0),
                     "malformed scenario rows must not shift the selected slice");
    }
    std::ofstream(scen_file) << "version 1\n0\taudit-regression.map\t99\t1\t0\t0\t2\t0\t2\n";
    ok &= expect(!load_problem(map_file, scen_file, 1, 0), "scenario/map dimensions must match");
    write_inputs(1);
    for (const char* malformed : {
            "type octile\nheight 2\nwidth 3\nmap\n.\n...\n",
            "type octile\nheight 2\nwidth 3\nmap\n...\n",
            "type octile\nheight 1\nwidth 3\nmap\n...\n...\n",
            "type octile\nheight 0\nwidth 3\nmap\n...\n",
            "type octile\nheight 1x\nwidth 3\nmap\n...\n",
            "garbage\nheight 1\nwidth 3\nmap\n...\n"}) {
        std::ofstream(map_file) << malformed;
        result = LNS({map_file, scen_file, 1, 0}, config);
        ok &= expect(result.status == SolveStatus::InvalidInput && result.paths.empty(),
                     "malformed map must fail before search");
    }
    std::ofstream(map_file) << "type octile\r\nheight 1\r\nwidth 3\r\nmap\r\n...\r\n";
    ok &= expect(load_problem(map_file, scen_file, 1, 0).has_value(), "valid CRLF maps must load");
    const Grid ragged{{'.'}, {'.', '.', '.'}};
    ConflictZoneBuilder builder(ragged);
    ok &= expect(builder.build_reachable_zone({{1, 1}}, 1).empty(), "zone must not index ragged rows");
    bool rejected = false;
    try { MDDConstructor mdd(ragged, {1, 0}, {1, 2}, 2); }
    catch (const std::invalid_argument&) { rejected = true; }
    ok &= expect(rejected, "MDD must reject ragged rows");
    return ok;
}
bool saturated_zone() {
    std::ofstream(map_file) << "type octile\nheight 1\nwidth 7\nmap\n...@...\n";
    std::ofstream(scen_file) << "version 1\n"
        "0\taudit-regression.map\t7\t1\t0\t0\t2\t0\t2\n"
        "0\taudit-regression.map\t7\t1\t2\t0\t0\t0\t2\n";
    SolverConfig config;
    config.log_level = LogLevel::Quiet;
    config.makespan_increase_limit = 1;
    // No deadline: exhausting a disconnected component must advance makespans.
    const auto result = LNS({map_file, scen_file, 2, 0}, config);
    return expect(result.status == SolveStatus::Exhausted && result.paths.empty(),
                  "saturated disconnected zone must terminate");
}
bool last_slack_unit() {
    bool ok = true;
    for (bool dummy : {false, true}) {
        const Grid grid(4, std::vector<char>(4, '.'));
        std::vector<Position> starts{{1, 0}, {0, 1}}, goals{{1, 2}, {2, 2}};
        if (dummy) { starts.push_back({3, 3}); goals.push_back({3, 3}); }
        CurrentSolution solution(4, 4, 3, static_cast<int>(starts.size()), starts, goals);
        solution.agent_paths[0] = {{1, 0}, {1, 1}, {1, 2}, {1, 2}};
        solution.agent_paths[1] = {{0, 1}, {1, 1}, {2, 1}, {2, 2}};
        if (dummy) solution.agent_paths[2] = {{3, 3}, {3, 3}, {3, 3}, {3, 3}};
        solution.calculate_waiting_times(goals, 3);
        solution.create_path_map();
        auto witness = solution.agent_paths;
        witness[0] = {{1, 0}, {1, 0}, {1, 1}, {1, 2}};
        ok &= expect(mapf::verify_solution(witness, starts, goals, grid).valid(), "one-wait witness must be valid");
        std::set<Position> zone{{1, 0}, {1, 1}, {1, 2}, {0, 1}, {2, 1}};
        Grid masked = grid;
        for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c)
            if (!zone.count({r, c})) masked[r][c] = '@';
        std::mt19937 rng(7);
        const auto result = lazy_solve_with_waiting_time(
            solution, grid, masked, zone, {}, {}, {}, 0, 2, 0, 0, rng);
        ok &= expect(result.solved() && result.waiting_attempts.size() == 2,
                     "last slack unit must be tried regardless of unrelated agents");
        ok &= expect(mapf::verify_solution(solution.agent_paths, starts, goals, grid).valid(),
                     "last-unit repair must be independently valid");
        ok &= expect(solution.get_waiting_time(0) == 0 && solution.get_waiting_time(1) == 0,
                     "repair must consume precisely one unit of available slack");
        if (dummy) ok &= expect(solution.agent_paths.at(2) == witness.at(2) && solution.get_waiting_time(2) == 3,
                               "unrelated path and budget must remain unchanged");
    }
    return ok;
}
class RecordingSolver final : public SatSolver {
public:
    std::unique_ptr<SatSolver> backend = make_sat_solver();
    std::vector<std::vector<int>> seen;
    void set_deadline(SolverDeadline d) override { backend->set_deadline(d); }
    SatOperationResult reset() override { return backend->reset(); }
    SatOperationResult add_clause(const SatClause& c) override { return backend->add_clause(c); }
    SatSolveResult solve() override { return backend->solve(); }
    SatSolveResult solve(const SatAssumptions& a) override { seen.push_back(a.literals); return backend->solve(a); }
    const std::vector<int>& model() const override { return backend->model(); }
};
bool path_assumptions() {
    bool ok = true;
    for (int entry : {0, 5}) {
        const Grid grid(3, std::vector<char>(3, '.'));
        MDDConstructor a(grid, {1, 0}, {1, 2}, 2), b(grid, {0, 1}, {2, 1}, 2);
        std::unordered_map<int, std::shared_ptr<MDD>> mdds{{0, a.construct_mdd()}, {1, b.construct_mdd()}};
        for (auto& item : mdds) align_mdd_to_time_window(item.second, entry, entry + 2, entry, entry + 2);
        CNFConstructor constructor(mdds, true);
        auto cnf = constructor.construct_cnf();
        RecordingSolver solver;
        const auto result = lazy_SAT_solve(solver, cnf, constructor,
            {{0, {entry, entry + 2}}, {1, {entry, entry + 2}}}, entry, entry + 2, 3);
        ok &= expect(result.status == SolveStatus::Exhausted, "forced crossing must remain UNSAT");
        ok &= expect(solver.seen.size() == 1, "real path assumptions must reach adapter");
        if (solver.seen.size() == 1) {
            auto literals = solver.seen.front(); std::sort(literals.begin(), literals.end());
            ok &= expect(literals == std::vector<int>({1, 2, 3, 4, 5, 6}),
                         "six path literal IDs must survive conversion and absolute time offsets");
        }
    }
    return ok;
}
bool validator_boundaries() {
    const Grid grid(2, std::vector<char>(5, '.'));
    CurrentSolution solution(2, 5, 4, 1, {{0, 0}}, {{0, 4}});
    solution.agent_paths[0] = {{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}};
    solution.agent_waiting_time[0] = 0; solution.create_path_map();
    const auto valid = build_local_problem_for_zone(solution, {{0, 1}, {0, 2}, {0, 3}}, grid, grid, {}, {}, 0, 0, 4);
    bool ok = expect(validate_local_zone_state(valid, solution).valid, "baseline local state must validate");
    auto broken = valid;
    broken.segments[0].mdd = valid.segments[0].mdd->copy();
    broken.segments[0].mdd->levels.begin()->second.front()->children.clear();
    ok &= expect(!validate_local_zone_state(broken, solution).valid, "MDD dead end must be rejected before SAT");
    broken = valid;
    broken.segments[0].path = {{1, 1}, {1, 2}, {1, 3}};
    MDDConstructor displaced(grid, {1, 1}, {1, 3}, 2);
    broken.segments[0].mdd = displaced.construct_mdd();
    align_mdd_to_time_window(broken.segments[0].mdd, 1, 3, 0, 4);
    ok &= expect(!validate_local_zone_state(broken, solution).valid,
                 "self-consistent MDD/path must still match locked global boundaries");
    return ok;
}
bool pseudo_agent_slack_round_trip() {
    const Grid grid(5, std::vector<char>(8, '.'));
    const Path first{{2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4}, {1, 4}, {1, 3},
                     {0, 3}, {0, 4}, {0, 4}, {0, 4}, {0, 4}, {0, 4}};
    const Path second{{0, 2}, {1, 2}, {2, 2}, {3, 2}, {4, 2}, {4, 3}, {4, 4},
                      {4, 5}, {4, 6}, {3, 6}, {2, 6}, {1, 6}, {0, 6}};
    CurrentSolution solution(5, 8, 12, 3, {{2, 0}, {0, 2}, {4, 0}}, {{0, 4}, {0, 6}, {4, 0}});
    solution.agent_paths = {{0, first}, {1, second}, {2, Path(13, {4, 0})}};
    solution.calculate_waiting_times(solution.goals, 12); solution.create_path_map();
    const std::set<Position> zone{{2, 1}, {2, 2}, {2, 3}, {1, 2}, {3, 2}, {1, 3}};
    Grid masked = grid;
    for (int r = 0; r < 5; ++r) for (int c = 0; c < 8; ++c)
        if (!zone.count({r, c})) masked[r][c] = '@';
    const auto state = build_local_problem_for_zone(solution, zone, masked, grid, {}, {}, 0, 0, 6);
    bool ok = expect(state.original_to_segments.at(0).size() == 2,
                     "round-trip fixture must contain a real segment and re-entry pseudo agent");
    std::mt19937 rng(7);
    const auto result = lazy_solve_with_waiting_time(solution, grid, masked, zone, {}, {}, {}, 0, 6, 0, 0, rng);
    ok &= expect(result.solved(), "pseudo-agent repair with slack and refresh must solve");
    ok &= expect(std::any_of(result.waiting_attempts.begin(), result.waiting_attempts.end(),
        [](const auto& metric) { return metric.applied_waiting_time && metric.extended_time_window; }),
        "round trip must exercise actual slack allocation and time-window refresh");
    ok &= expect(mapf::verify_solution(solution.agent_paths, solution.starts, solution.goals, grid).valid(),
                 "reassembled pseudo-agent solution must pass independent verification");
    const auto& repaired = solution.agent_paths.at(0);
    ok &= expect(repaired.size() == first.size() && repaired.front() == first.front(),
                 "round trip must retain the global horizon and prefix");
    // One authorized wait shifts the intervening outside-zone path and later
    // re-entry by one timestep. Their spatial sequence must remain intact.
    for (std::size_t old_t = 4; old_t < first.size() - 1; ++old_t)
        ok &= expect(repaired[old_t + 1] == first[old_t], "outside sequence or shifted re-entry was changed");
    ok &= expect(solution.agent_paths.at(1) == second && solution.agent_paths.at(2) == Path(13, {4, 0}),
                 "other real-agent paths must be preserved");
    ok &= expect(solution.get_waiting_time(0) == 3 && solution.get_waiting_time(1) == 0 &&
                 solution.get_waiting_time(2) == 12, "round trip must charge only the delayed real agent once");
    return ok;
}
}
int main() {
    bool ok = input_contract();
    ok &= saturated_zone();
    ok &= last_slack_unit();
    ok &= path_assumptions();
    ok &= validator_boundaries();
    ok &= pseudo_agent_slack_round_trip();
    std::remove(map_file); std::remove(scen_file);
    if (ok) std::cout << "PASS: audit correctness regressions\n";
    return ok ? 0 : 1;
}
