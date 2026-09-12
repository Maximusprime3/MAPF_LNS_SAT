#include "lnssat/LNS.h"
#include "lnssat/Lazy_SAT_Solve.h"
#include "lnssat/cnf/CNF.h"
#include "lnssat/cnf/CNFConstructor.h"

#include <iostream>
#include <unordered_map>
#include <string>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << std::endl;
    }
    return condition;
}

}  // namespace

int main() {
    bool ok = true;

    const auto default_variant = parse_neighborhood_variant("lns-sat");
    ok &= expect(default_variant.has_value() &&
                     *default_variant == NeighborhoodVariant::LnsSat,
                 "the public default variant should resolve to LNS-SAT");

    const LNSResult default_result = LNS(
        SolveRequest{"tests/does-not-exist.map",
                     "tests/does-not-exist.scen", 1, 0},
        SolverConfig{});
    ok &= expect(default_result.status == SolveStatus::InvalidInput,
                 "missing inputs should fail as InvalidInput before search");
    ok &= expect(default_result.seed == 42,
                 "the established default seed should be preserved in LNSResult");
    ok &= expect(default_result.paths.empty() && default_result.runtime_ms == 0.0,
                 "invalid input should not produce paths or measured search time");

    SolverConfig overridden_config;
    overridden_config.seed = 73;
    overridden_config.neighborhood_variant =
        NeighborhoodVariant::FixedStep2;
    const LNSResult overridden_seed = LNS(
        SolveRequest{"tests/does-not-exist.map",
                     "tests/does-not-exist.scen", 1, 0},
        overridden_config);
    ok &= expect(overridden_seed.status == SolveStatus::InvalidInput &&
                     overridden_seed.seed == 73,
                 "an explicit seed should propagate before input loading");

    std::unordered_map<int, std::shared_ptr<MDD>> empty_mdds;
    CNFConstructor constructor(empty_mdds, true);
    CNF cnf;
    const LazySolveResult zero_iterations = lazy_SAT_solve(
        cnf, constructor, {}, 0, 0, 0);
    ok &= expect(zero_iterations.status == SolveStatus::Exhausted,
                 "the low-level lazy solver currently treats zero iterations as exhaustion");
    ok &= expect(zero_iterations.metrics.iterations.empty(),
                 "zero lazy iterations should not invoke the SAT loop");

    if (!ok) {
        return 1;
    }
    std::cout << "PASS: established solver configuration behavior" << std::endl;
    return 0;
}
