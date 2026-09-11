#include "../Lazy_SAT_Solve.h"
#include "../SolutionVerifier.h"
#include "../../mdd/MDDConstructor.h"
#include <array>
#include <cstdlib>
#include <iostream>
#include <set>
#include <streambuf>

namespace {
// Independent joint-state enumeration on a 2x2 square. This oracle uses no
// MDD, CNF, solver collision helper, or production grid helper.
int distance(int a, int b) {
    return std::abs(a / 2 - b / 2) + std::abs(a % 2 - b % 2);
}
bool feasible(std::pair<int, int> starts, std::pair<int, int> goals, int horizon) {
    std::set<std::pair<int, int>> states{starts};
    for (int t = 0; t < horizon; ++t) {
        std::set<std::pair<int, int>> next;
        for (const auto& [a, b] : states) {
            for (int u = 0; u < 4; ++u) for (int v = 0; v < 4; ++v) {
                if (distance(a, u) > 1 || distance(b, v) > 1) continue;
                if (u == v || (u == b && v == a)) continue;
                next.emplace(u, v);
            }
        }
        states = std::move(next);
    }
    return states.count(goals) != 0;
}
class QuietBuffer : public std::streambuf {
    int overflow(int ch) override { return traits_type::not_eof(ch); }
};
}
int main() {
    QuietBuffer quiet;
    auto* output = std::cout.rdbuf(&quiet);
    const std::vector<std::vector<char>> grid(2, std::vector<char>(2, '.'));
    auto position = [](int cell) { return std::make_pair(cell / 2, cell % 2); };
    int cases = 0;
    bool ok = true;
    for (int a = 0; a < 4; ++a) for (int b = 0; b < 4; ++b) {
        if (a == b) continue;
        for (int u = 0; u < 4; ++u) for (int v = 0; v < 4; ++v) {
            if (u == v) continue;
            const int lower_bound = std::max(distance(a, u), distance(b, v));
            for (int horizon = lower_bound; horizon <= 4; ++horizon) {
                ++cases;
                const bool expected = feasible({a, b}, {u, v}, horizon);
                MDDConstructor first(grid, position(a), position(u), horizon);
                MDDConstructor second(grid, position(b), position(v), horizon);
                CNFConstructor constructor({{0, first.construct_mdd()}, {1, second.construct_mdd()}}, true);
                auto cnf = constructor.construct_cnf();
                const auto result = lazy_SAT_solve(cnf, constructor,
                    {{0, {0, horizon}}, {1, {0, horizon}}}, 0, horizon, 1000);
                const bool valid = !result.solved() || mapf::verify_solution(result.local_paths,
                    {position(a), position(b)}, {position(u), position(v)}, grid).valid();
                if (result.solved() != expected || !valid ||
                    (result.status != SolveStatus::Solved && result.status != SolveStatus::Exhausted)) {
                    std::cerr << "FAIL: oracle disagreement starts=" << a << ',' << b
                              << " goals=" << u << ',' << v << " horizon=" << horizon << '\n';
                    ok = false;
                }
            }
        }
    }
    std::cout.rdbuf(output);
    if (ok) std::cout << "PASS: " << cases << " fixed-horizon MAPF cases match independent enumeration\n";
    return ok ? 0 : 1;
}
