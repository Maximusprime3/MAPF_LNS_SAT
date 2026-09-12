#include "lnssat/CurrentSolutionTransaction.h"

#include <iostream>
#include <utility>
#include <vector>

namespace {

CurrentSolution make_solution() {
    CurrentSolution solution(
        2, 2, 2, 1,
        std::vector<std::pair<int, int>>{{0, 0}},
        std::vector<std::pair<int, int>>{{0, 1}});
    solution.agent_paths[0] = {{0, 0}, {0, 1}, {0, 1}};
    solution.agent_waiting_time[0] = 1;
    solution.create_path_map();
    return solution;
}

bool transaction_rolls_back_on_early_return(CurrentSolution& solution) {
    CurrentSolutionTransaction transaction(solution);
    solution.agent_paths[0] = {{0, 0}, {1, 0}, {1, 1}};
    solution.agent_waiting_time[0] = 0;
    solution.create_path_map();
    // Returning without commit models every unsuccessful early exit from the
    // waiting-time solver. The transaction destructor must restore state.
    return false;
}

}  // namespace

int main() {
    CurrentSolution solution = make_solution();
    const auto original_paths = solution.agent_paths;
    const auto original_waiting_times = solution.agent_waiting_time;

    transaction_rolls_back_on_early_return(solution);
    if (solution.agent_paths != original_paths ||
        solution.agent_waiting_time != original_waiting_times ||
        solution.get_agents_at_position_time(0, 1, 1) != std::vector<int>{0}) {
        std::cerr << "FAIL: uncommitted transaction did not restore CurrentSolution" << std::endl;
        return 1;
    }

    {
        CurrentSolutionTransaction transaction(solution);
        solution.agent_paths[0] = {{0, 0}, {1, 0}, {1, 1}};
        solution.agent_waiting_time[0] = 0;
        solution.create_path_map();
        transaction.commit();
    }

    if (solution.agent_paths == original_paths || solution.agent_waiting_time[0] != 0) {
        std::cerr << "FAIL: committed transaction did not preserve CurrentSolution changes" << std::endl;
        return 1;
    }

    std::cout << "PASS: CurrentSolution transaction rollback and commit" << std::endl;
    return 0;
}
