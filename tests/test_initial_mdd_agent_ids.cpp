#include "lnssat/Lazy_SAT_Solve.h"
#include "lnssat/mdd/MDD.h"

#include <iostream>
#include <map>
#include <random>
#include <utility>
#include <vector>

int main() {
    // Agent 1 deliberately has no distance entry. The successful records must
    // remain IDs 0 and 2; returning bare MDD pointers would incorrectly relabel
    // agent 2 as agent 1 after the skipped construction.
    const std::vector<std::vector<char>> grid = {{'.', '.', '.'}};
    const std::vector<std::pair<int, int>> starts = {{0, 0}, {0, 1}, {0, 2}};
    const std::vector<std::pair<int, int>> goals = {{0, 1}, {0, 2}, {0, 1}};
    std::vector<std::map<std::pair<int, int>, int>> distances(3);
    distances[0][starts[0]] = 1;
    distances[2][starts[2]] = 1;

    const auto agent_mdds =
        create_mdds_with_waiting_time(grid, starts, goals, distances);
    if (agent_mdds.size() != 2 || agent_mdds[0].agent_id != 0 ||
        agent_mdds[1].agent_id != 2) {
        std::cerr << "FAIL: successful initial MDDs did not preserve agent IDs 0 and 2"
                  << std::endl;
        return 1;
    }

    // Confirm that each ID-keyed record also contains the MDD for that agent's
    // own start and goal, not merely the expected numeric label.
    std::mt19937 rng(42);
    for (const AgentMDD& agent_mdd : agent_mdds) {
        if (!agent_mdd.mdd) {
            std::cerr << "FAIL: agent " << agent_mdd.agent_id << " has a null MDD"
                      << std::endl;
            return 1;
        }
        const auto path = agent_mdd.mdd->sample_random_path(rng);
        if (path.empty() || path.front() != starts[agent_mdd.agent_id] ||
            path.back() != goals[agent_mdd.agent_id]) {
            std::cerr << "FAIL: MDD contents do not match agent " << agent_mdd.agent_id
                      << std::endl;
            return 1;
        }
    }

    std::cout << "PASS: initial MDD records preserve explicit agent IDs" << std::endl;
    return 0;
}
