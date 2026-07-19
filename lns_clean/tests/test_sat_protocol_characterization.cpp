#include "../Lazy_SAT_Solve.h"
#include "../../SATSolverManager.h"
#include "../../cnf/CNF.h"
#include "../../cnf/CNFConstructor.h"
#include "../../minisat/minisat-wrapper.h"

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << std::endl;
    }
    return condition;
}

bool clean_unsat(const MiniSatSolution& result) {
    return !result.satisfiable && result.error_message.empty();
}

}  // namespace

int main() {
    bool ok = true;

    MiniSatWrapper incremental;
    const MiniSatSolution prefix =
        incremental.solve_cnf_incremental({{1, 2}}, nullptr, false);
    ok &= expect(prefix.satisfiable && prefix.assignment.size() == 2,
                 "SAT prefix did not produce a two-variable model");

    const std::vector<int> preserving_assignment{1, 0};
    const MiniSatSolution preserving = incremental.solve_cnf_incremental(
        {{1, 2}}, &preserving_assignment, true);
    ok &= expect(preserving.satisfiable,
                 "compatible assumptions did not preserve SAT");

    const std::vector<int> conflicting_assignment{0, 0};
    const MiniSatSolution conflicting = incremental.solve_cnf_incremental(
        {{1, 2}}, &conflicting_assignment, true);
    ok &= expect(clean_unsat(conflicting),
                 "conflicting assumptions were not cleanly UNSAT");

    incremental.reset_incremental();
    const MiniSatSolution independent = incremental.solve_cnf_incremental(
        {{-1}}, nullptr, false);
    ok &= expect(independent.satisfiable &&
                     independent.assignment.size() == 1 &&
                     independent.assignment[0] == 0,
                 "reset did not start an independent incremental session");

    std::unordered_map<int, std::shared_ptr<MDD>> empty_mdds;
    CNFConstructor constructor(empty_mdds, true);

    CNF contradictory;
    contradictory.add_clause({1});
    contradictory.add_clause({-1});
    const LazySolveResult formula_unsat = lazy_SAT_solve(
        contradictory, constructor, {}, 0, 0, 1);
    ok &= expect(formula_unsat.status == SolveStatus::Exhausted &&
                     formula_unsat.message == "Local CNF is unsatisfiable" &&
                     formula_unsat.metrics.iterations.size() == 1,
                 "formula UNSAT was not preserved as bounded exhaustion");

    CNF empty;
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
    std::cout << "PASS: established incremental SAT and lazy outcome protocol"
              << std::endl;
    return 0;
}
