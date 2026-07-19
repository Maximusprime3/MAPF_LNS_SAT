#include "../Lazy_SAT_Solve.h"
#include "../SatSolver.h"
#include "../../cnf/CNF.h"
#include "../../cnf/CNFConstructor.h"

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << std::endl;
    }
    return condition;
}

class FakeSatSolver final : public SatSolver {
public:
    explicit FakeSatSolver(
        std::vector<SatSolveResult> scripted_results,
        std::vector<int> model = {1})
        : scripted_results_(std::move(scripted_results)),
          model_(std::move(model)) {}

    SatOperationResult reset() override {
        ++reset_calls;
        return {};
    }

    SatOperationResult add_clause(const SatClause& clause) override {
        added_batches.push_back({clause});
        return {};
    }

    SatOperationResult add_clauses(
        const std::vector<SatClause>& clauses) override {
        added_batches.push_back(clauses);
        return {};
    }

    SatSolveResult solve() override {
        ++plain_solve_calls;
        return next_result();
    }

    SatSolveResult solve(const SatAssumptions&) override {
        ++assumption_solve_calls;
        return next_result();
    }

    const std::vector<int>& model() const override {
        return model_;
    }

    int reset_calls = 0;
    int plain_solve_calls = 0;
    int assumption_solve_calls = 0;
    std::vector<std::vector<SatClause>> added_batches;

private:
    SatSolveResult next_result() {
        if (next_result_ >= scripted_results_.size()) {
            return SatSolveResult{
                SatResultKind::Error, {}, "unscripted fake call"};
        }
        return scripted_results_[next_result_++];
    }

    std::vector<SatSolveResult> scripted_results_;
    std::size_t next_result_ = 0;
    std::vector<int> model_;
};

SatSolveResult scripted(
    SatResultKind kind,
    int decisions = 0,
    int propagations = 0,
    double seconds = 0.0,
    std::string diagnostic = {}) {
    return SatSolveResult{
        kind,
        SatStatistics{decisions, propagations, seconds},
        std::move(diagnostic)};
}

}  // namespace

int main() {
    bool ok = true;

    auto real = make_sat_solver();
    ok &= expect(real->reset().ok, "real adapter reset failed");
    ok &= expect(real->add_clause({1, 2}).ok,
                 "real adapter clause addition failed");
    const SatSolveResult sat = real->solve();
    ok &= expect(sat.kind == SatResultKind::Sat &&
                     real->model().size() == 2,
                 "SAT classification/model extraction failed");

    ok &= expect(real->solve(SatAssumptions{{1, -2}}).kind ==
                     SatResultKind::Sat,
                 "compatible assumptions did not preserve SAT");
    ok &= expect(real->solve(SatAssumptions{{-1, -2}}).kind ==
                     SatResultKind::Unsat,
                 "conflicting assumptions were not UNSAT");

    ok &= expect(real->reset().ok && real->add_clause({-1}).ok,
                 "independent reset setup failed");
    ok &= expect(real->solve().kind == SatResultKind::Sat &&
                     real->model().size() == 1 &&
                     real->model()[0] == 0,
                 "reset did not create an independent session");

    FakeSatSolver incremental(
        {scripted(SatResultKind::Sat),
         scripted(SatResultKind::Sat),
         scripted(SatResultKind::Sat)});
    std::size_t loaded = 0;
    const std::vector<SatClause> prefix{{1}};
    solve_sat_iteration(incremental, prefix, loaded, nullptr, true);
    const std::vector<SatClause> extended{{1}, {2}};
    solve_sat_iteration(incremental, extended, loaded, nullptr, false);
    solve_sat_iteration(incremental, extended, loaded, nullptr, false);
    ok &= expect(incremental.added_batches.size() == 2 &&
                     incremental.added_batches[0] ==
                         std::vector<SatClause>{{1}} &&
                     incremental.added_batches[1] ==
                         std::vector<SatClause>{{2}},
                 "unchanged clause prefixes were re-added");

    FakeSatSolver fallback(
        {scripted(SatResultKind::Unsat, 2, 3, 0.004),
         scripted(SatResultKind::Sat, 5, 7, 0.006)});
    loaded = 1;
    const SatAssumptions previous_model{{1}};
    const SatIterationResult fallback_result = solve_sat_iteration(
        fallback, prefix, loaded, &previous_model, false);
    ok &= expect(fallback_result.kind == SatResultKind::Sat &&
                     fallback.reset_calls == 1 &&
                     fallback.assumption_solve_calls == 1 &&
                     fallback.plain_solve_calls == 1 &&
                     fallback_result.solver_calls == 2,
                 "assumption UNSAT did not cause exactly one reset/retry");
    ok &= expect(fallback_result.statistics.decisions == 7 &&
                     fallback_result.statistics.propagations == 10 &&
                     fallback_result.statistics.solve_time_seconds > 0.009 &&
                     fallback_result.statistics.solve_time_seconds < 0.011,
                 "fallback metrics did not include both solver calls");

    FakeSatSolver backend_error(
        {scripted(SatResultKind::Error, 0, 0, 0.0,
                  "injected backend failure")});
    loaded = 1;
    const SatIterationResult error_result = solve_sat_iteration(
        backend_error, prefix, loaded, &previous_model, false);
    ok &= expect(error_result.kind == SatResultKind::Error &&
                     backend_error.reset_calls == 0 &&
                     backend_error.assumption_solve_calls == 1 &&
                     backend_error.plain_solve_calls == 0,
                 "backend error was hidden by the UNSAT retry");

    std::unordered_map<int, std::shared_ptr<MDD>> empty_mdds;
    CNFConstructor constructor(empty_mdds, true);

    FakeSatSolver lazy_error(
        {scripted(SatResultKind::Error, 0, 0, 0.0,
                  "lazy fake failure")});
    CNF empty;
    const LazySolveResult propagated_error = lazy_SAT_solve(
        lazy_error, empty, constructor, {}, 0, 0, 1);
    ok &= expect(propagated_error.status == SolveStatus::InvalidState &&
                     propagated_error.message.find("lazy fake failure") !=
                         std::string::npos &&
                     lazy_error.reset_calls == 1 &&
                     lazy_error.plain_solve_calls == 1,
                 "lazy backend error did not propagate immediately");

    CNF contradictory;
    contradictory.add_clause({1});
    contradictory.add_clause({-1});
    const LazySolveResult formula_unsat = lazy_SAT_solve(
        contradictory, constructor, {}, 0, 0, 1);
    ok &= expect(formula_unsat.status == SolveStatus::Exhausted &&
                     formula_unsat.message ==
                         "Local CNF is unsatisfiable" &&
                     formula_unsat.metrics.iterations.size() == 1,
                 "formula UNSAT was not bounded exhaustion");

    const LazySolveResult iteration_exhausted = lazy_SAT_solve(
        empty, constructor, {}, 0, 0, 0);
    ok &= expect(iteration_exhausted.status == SolveStatus::Exhausted &&
                     iteration_exhausted.message ==
                         "Lazy SAT iteration limit reached" &&
                     iteration_exhausted.metrics.iterations.empty(),
                 "iteration exhaustion was not distinct from formula UNSAT");

    if (!ok) {
        return 1;
    }
    std::cout << "PASS: typed incremental SAT and lazy call protocol"
              << std::endl;
    return 0;
}
