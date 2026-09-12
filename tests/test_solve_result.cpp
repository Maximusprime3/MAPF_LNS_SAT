#include "lnssat/LNS.h"
#include "lnssat/Lazy_SAT_Solve.h"
#include "lnssat/Solve_Local_Zone.h"
#include "lnssat/Waiting_time_Solve.h"

#include <iostream>
#include <string>

int main() {
    bool passed = true;
    passed &= std::string(solve_status_name(SolveStatus::Solved)) == "solved";
    passed &= std::string(solve_status_name(SolveStatus::Exhausted)) == "exhausted";
    passed &= std::string(solve_status_name(SolveStatus::InvalidInput)) == "invalid_input";
    passed &= std::string(solve_status_name(SolveStatus::InvalidState)) == "invalid_state";
    passed &= solve_status_exit_code(SolveStatus::Solved) == 0;
    passed &= solve_status_exit_code(SolveStatus::Exhausted) == 1;
    passed &= solve_status_exit_code(SolveStatus::InvalidInput) == 2;
    passed &= solve_status_exit_code(SolveStatus::InvalidState) == 3;

    // Every internal boundary defaults to a safe exhausted outcome. None may
    // accidentally report success merely because a result was default-built.
    const LazySolveResult lazy_default;
    const WaitingSolveResult waiting_default;
    const LocalZoneResult zone_default;
    passed &= lazy_default.status == SolveStatus::Exhausted && !lazy_default.solved();
    passed &= waiting_default.status == SolveStatus::Exhausted && !waiting_default.solved();
    passed &= zone_default.status == SolveStatus::Exhausted && !zone_default.solved();

    // Missing files exercise the public input-error path without executing the
    // search. Failure must be explicit rather than encoded as an empty map.
    const LNSResult invalid_input = LNS(
        SolveRequest{"tests/does-not-exist.map",
                     "tests/does-not-exist.scen", 1, 0},
        SolverConfig{});
    passed &= invalid_input.status == SolveStatus::InvalidInput;
    passed &= !invalid_input.solved();
    passed &= invalid_input.paths.empty();
    passed &= !invalid_input.message.empty();

    SolveRequest request{
        "tests/fixtures/all-terrain.map",
        "tests/fixtures/all-terrain-valid.scen",
        1,
        0};
    SolverConfig invalid_config;
    invalid_config.seed = 73;
    invalid_config.neighborhood_variant = NeighborhoodVariant::FixedStep2;
    invalid_config.lazy_iteration_limit = 0;
    const LNSResult invalid_config_result = LNS(request, invalid_config);
    passed &= invalid_config_result.status == SolveStatus::InvalidInput;
    passed &= !invalid_config_result.search_started;
    passed &= invalid_config_result.paths.empty();
    passed &= invalid_config_result.runtime_ms == 0.0;
    passed &= invalid_config_result.seed == 73;
    passed &= invalid_config_result.neighborhood_variant ==
              NeighborhoodVariant::FixedStep2;

    SolverConfig resolved_config;
    resolved_config.seed = 91;
    resolved_config.neighborhood_variant =
        NeighborhoodVariant::IncreasingStep;
    const LNSResult resolved_metadata = LNS(
        SolveRequest{"tests/does-not-exist.map",
                     "tests/does-not-exist.scen", 1, 0},
        resolved_config);
    passed &= resolved_metadata.status == SolveStatus::InvalidInput;
    passed &= resolved_metadata.search_started;
    passed &= resolved_metadata.seed == 91;
    passed &= resolved_metadata.neighborhood_variant ==
              NeighborhoodVariant::IncreasingStep;

    if (!passed) {
        std::cerr << "FAIL: structured LNS result/status contract" << std::endl;
        return 1;
    }
    std::cout << "PASS: structured LNS result/status contract" << std::endl;
    return 0;
}
