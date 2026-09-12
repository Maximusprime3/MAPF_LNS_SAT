#include "lnssat/Lazy_SAT_Solve.h"
#include "lnssat/Waiting_time_Solve.h"
#include "lnssat/mdd/MDDConstructor.h"
#include <chrono>
#include <iostream>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
bool expect(bool condition, const char* message) {
    if (!condition) std::cerr << "FAIL: " << message << '\n';
    return condition;
}
class LateSolver final : public SatSolver {
public:
    SolverDeadline deadline;
    SatResultKind answer = SatResultKind::Sat;
    int calls = 0;
    std::vector<int> values{1};
    void set_deadline(SolverDeadline d) override { deadline = d; }
    SatOperationResult reset() override { return {}; }
    SatOperationResult add_clause(const SatClause&) override { return {}; }
    SatSolveResult solve() override {
        ++calls;
        if (deadline) std::this_thread::sleep_until(*deadline + 1ms);
        return {answer, {}, {}};
    }
    SatSolveResult solve(const SatAssumptions&) override { return solve(); }
    const std::vector<int>& model() const override { return values; }
};
}
int main() {
    bool ok = true;
    for (auto answer : {SatResultKind::Sat, SatResultKind::Unsat}) {
        LateSolver late; late.answer = answer;
        std::size_t loaded = 0;
        const SatAssumptions assumptions{{1}};
        const auto result = solve_sat_iteration(late, {{1}}, loaded, &assumptions, true, Clock::now() + 5ms);
        ok &= expect(result.kind == SatResultKind::Interrupted && result.model.empty(),
                     "late SAT/UNSAT must become interruption without a model");
        ok &= expect(late.calls == 1, "late assumption UNSAT must not retry without limits");
    }
    LateSolver never_called;
    std::size_t loaded = 0;
    auto result = solve_sat_iteration(never_called, {{1}}, loaded, nullptr, true, Clock::now() - 1ms);
    ok &= expect(result.kind == SatResultKind::Interrupted && never_called.calls == 0,
                 "expired deadline must skip backend invocation");

    // An unsatisfiable pigeonhole instance that cannot be discharged by a
    // trivial unit propagation. Cancellation must happen inside one solve.
    auto real = make_sat_solver();
    constexpr int pigeons = 20, holes = 19;
    auto var = [](int p, int h) { return 1 + p * holes + h; };
    for (int p = 0; p < pigeons; ++p) {
        SatClause one_hole;
        for (int h = 0; h < holes; ++h) one_hole.push_back(var(p, h));
        real->add_clause(one_hole);
    }
    for (int h = 0; h < holes; ++h)
        for (int p = 0; p < pigeons; ++p)
            for (int q = p + 1; q < pigeons; ++q)
                real->add_clause({-var(p, h), -var(q, h)});
    const auto start = Clock::now();
    real->set_deadline(start + 5ms);
    const auto interrupted = real->solve();
    ok &= expect(interrupted.kind == SatResultKind::Interrupted && real->model().empty(),
                 "MiniSAT must distinguish deadline interruption from UNSAT");
    ok &= expect(Clock::now() - start < 2s, "MiniSAT must return promptly after deadline");
    real->set_deadline({}); real->reset(); real->add_clause({1});
    ok &= expect(real->solve().kind == SatResultKind::Sat, "reset session must remain usable after interruption");

    const std::vector<std::vector<char>> grid(3, std::vector<char>(3, '.'));
    MDDConstructor mdd(grid, {0, 0}, {2, 2}, 1000, {}, Clock::now() - 1ms);
    ok &= expect(!mdd.construct_mdd(), "expired initial/local MDD construction must stop");
    CurrentSolution solution(3, 3, 3, 1, {{0, 0}}, {{0, 2}});
    solution.agent_paths[0] = {{0, 0}, {0, 1}, {0, 2}, {0, 2}};
    solution.agent_waiting_time[0] = 1; solution.create_path_map();
    const auto paths = solution.agent_paths;
    const auto budgets = solution.agent_waiting_time;
    const auto occupancy = solution.path_map;
    std::mt19937 rng(7);
    auto waiting = lazy_solve_with_waiting_time(solution, grid, grid,
        {{0, 0}, {0, 1}, {0, 2}}, {}, {}, {}, 0, 3, 0, 0, rng, SolverConfig{}, Clock::now() - 1ms);
    ok &= expect(waiting.status == SolveStatus::Exhausted && solution.agent_paths == paths &&
                     solution.agent_waiting_time == budgets && solution.path_map == occupancy,
                 "deadline rejection must preserve paths, budgets, and occupancy");
    if (ok) std::cout << "PASS: cooperative deadlines and interruption propagation\n";
    return ok ? 0 : 1;
}
