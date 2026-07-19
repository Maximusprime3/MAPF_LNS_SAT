#include "../LNS.h"
#include "../Lazy_SAT_Solve.h"
#include "../Solve_Local_Zone.h"
#include "../Waiting_time_Solve.h"

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
        "tests/does-not-exist.map",
        "tests/does-not-exist.scen",
        1,
        0,
        true,
        42,
        NeighborhoodVariant::LnsSat);
    passed &= invalid_input.status == SolveStatus::InvalidInput;
    passed &= !invalid_input.solved();
    passed &= invalid_input.paths.empty();
    passed &= !invalid_input.message.empty();

    if (!passed) {
        std::cerr << "FAIL: structured LNS result/status contract" << std::endl;
        return 1;
    }
    std::cout << "PASS: structured LNS result/status contract" << std::endl;
    return 0;
}
