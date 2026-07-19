#include "../SatSolver.h"

#include <iostream>

namespace {

bool clean_unsat(const SatSolveResult& result) {
    return result.kind == SatResultKind::Unsat &&
           result.diagnostic.empty();
}

}  // namespace

int main() {
    auto direct = make_sat_solver();
    if (!direct->reset().ok ||
        !direct->add_clauses({{1}, {-1}}).ok) {
        std::cerr << "FAIL: could not prepare direct contradiction"
                  << std::endl;
        return 1;
    }
    if (!clean_unsat(direct->solve())) {
        std::cerr << "FAIL: direct contradiction was not reported as clean UNSAT"
                  << std::endl;
        return 1;
    }

    auto incremental = make_sat_solver();
    if (!incremental->reset().ok ||
        !incremental->add_clause({1}).ok) {
        std::cerr << "FAIL: could not prepare satisfiable prefix"
                  << std::endl;
        return 1;
    }
    if (incremental->solve().kind != SatResultKind::Sat) {
        std::cerr << "FAIL: satisfiable incremental prefix did not solve cleanly"
                  << std::endl;
        return 1;
    }
    if (!incremental->add_clause({-1}).ok ||
        !clean_unsat(incremental->solve())) {
        std::cerr << "FAIL: incremental contradiction was not reported as clean UNSAT"
                  << std::endl;
        return 1;
    }

    std::cout << "PASS: MiniSAT contradictions are classified as UNSAT" << std::endl;
    return 0;
}
