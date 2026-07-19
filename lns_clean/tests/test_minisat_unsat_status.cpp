#include "../../SATSolverManager.h"
#include "../../minisat/minisat-wrapper.h"

#include <iostream>
#include <vector>

namespace {

bool is_clean_unsat(const MiniSatSolution& result) {
    return !result.satisfiable && result.error_message.empty();
}

}  // namespace

int main() {
    // A direct contradictory formula is a normal UNSAT conclusion, not a
    // backend failure.
    MiniSatWrapper direct_wrapper;
    const std::vector<std::vector<int>> contradictory = {{1}, {-1}};
    const MiniSatSolution direct = direct_wrapper.solve_cnf(contradictory);
    if (!is_clean_unsat(direct)) {
        std::cerr << "FAIL: direct contradiction was not reported as clean UNSAT"
                  << std::endl;
        return 1;
    }

    // Reproduce the path used by lazy SAT: solve a satisfiable prefix, then add
    // a clause that makes the incremental solver contradictory.
    MiniSatWrapper incremental_wrapper;
    const MiniSatSolution prefix =
        incremental_wrapper.solve_cnf_incremental({{1}}, nullptr, false);
    if (!prefix.satisfiable || !prefix.error_message.empty()) {
        std::cerr << "FAIL: satisfiable incremental prefix did not solve cleanly"
                  << std::endl;
        return 1;
    }
    const MiniSatSolution incremental =
        incremental_wrapper.solve_cnf_incremental(contradictory, nullptr, false);
    if (!is_clean_unsat(incremental)) {
        std::cerr << "FAIL: incremental contradiction was not reported as clean UNSAT"
                  << std::endl;
        return 1;
    }

    std::cout << "PASS: MiniSAT contradictions are classified as UNSAT" << std::endl;
    return 0;
}
