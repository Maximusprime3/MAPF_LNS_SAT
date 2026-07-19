#include "../Load_LNSProblem.h"
#include "../Local_Zone.h"
#include "../NeighborhoodVariant.h"
#include "../SolutionVerifier.h"
#include "../Solve_Local_Zone.h"
#include "../../SATSolverManager.h"
#include "../../mdd/MDDConstructor.h"

#include <array>
#include <iostream>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using Grid = std::vector<std::vector<char>>;
using Position = std::pair<int, int>;

constexpr std::array<char, 7> kMovingAiTerrain{{'.', 'G', '@', 'O', 'T', 'S', 'W'}};

class TestRunner {
public:
    void expect(bool condition, const std::string& message) {
        ++checks_;
        if (!condition) {
            ++failures_;
            std::cerr << "FAIL: " << message << std::endl;
        }
    }

    int finish() const {
        if (failures_ == 0) {
            std::cout << "PASS: " << checks_
                      << " deterministic grid characterization checks" << std::endl;
            return 0;
        }
        std::cerr << "FAIL: " << failures_ << " of " << checks_
                  << " grid characterization checks failed" << std::endl;
        return 1;
    }

private:
    int checks_ = 0;
    int failures_ = 0;
};

bool mdd_accepts(char terrain) {
    try {
        MDDConstructor constructor({{terrain}}, {0, 0}, {0, 0}, 0);
        return constructor.construct_mdd() != nullptr;
    } catch (const std::invalid_argument&) {
        return false;
    }
}

bool verifier_accepts(char terrain) {
    return mapf::verify_path_geometry({{0, 0}}, {{terrain}}).valid();
}

bool has_issue(
    const mapf::VerificationReport& report,
    mapf::VerificationIssueCode expected) {
    for (const auto& issue : report.issues) {
        if (issue.code == expected) {
            return true;
        }
    }
    return false;
}

LocalZoneResult characterize_goal_only_fallback() {
    const Grid grid{{'G'}};
    const Position only_cell{0, 0};
    CurrentSolution solution(1, 1, 0, 1, {only_cell}, {only_cell});
    solution.agent_paths[0] = {only_cell};
    solution.calculate_waiting_times({only_cell}, 0);
    solution.create_path_map();

    DiamondBucket bucket;
    bucket.positions = {only_cell};
    bucket.earliest_t = 0;
    bucket.latest_t = 0;

    const std::vector<std::vector<std::vector<int>>> conflict_map(
        1, std::vector<std::vector<int>>(1));
    std::mt19937 rng(7);
    return solve_local_zone(
        grid,
        bucket,
        {},
        conflict_map,
        solution,
        neighborhood_policy(NeighborhoodVariant::LnsSat),
        0,
        rng,
        "grid-characterization",
        0);
}

}  // namespace

int main() {
    TestRunner tests;

    const std::string map_path = "tests/fixtures/all-terrain.map";
    const auto loaded_grid = SATSolverManager::load_map(map_path);
    tests.expect(loaded_grid.size() == 1 && loaded_grid.front().size() == 8,
                 "loader should preserve the complete fixture row");
    if (loaded_grid.size() == 1 && loaded_grid.front().size() == 8) {
        for (std::size_t index = 0; index < kMovingAiTerrain.size(); ++index) {
            tests.expect(
                loaded_grid.front()[index] == kMovingAiTerrain[index],
                std::string("loader changed accepted terrain symbol ") +
                    kMovingAiTerrain[index]);
        }
        tests.expect(loaded_grid.front().back() == 'X',
                     "permissive loader should preserve unknown terrain bytes");
    }

    // Characterize loader validation separately from terrain preservation.
    tests.expect(
        load_problem(
            map_path, "tests/fixtures/all-terrain-valid.scen", 1, 0).has_value(),
        "problem loader should accept in-bounds walkable endpoints");
    tests.expect(
        load_problem(
            map_path, "tests/fixtures/all-terrain-blocked.scen", 1, 0).has_value(),
        "current problem loader accepts blocked endpoints for downstream validation");
    tests.expect(
        load_problem(
            map_path,
            "tests/fixtures/all-terrain-out-of-bounds.scen",
            1,
            0).has_value(),
        "current problem loader accepts out-of-grid endpoints for downstream validation");

    // Record the current per-component terrain rules before centralization.
    const std::set<char> verifier_walkable{'.', 'G', 'S'};
    const std::set<char> mdd_walkable{'.', 'G'};
    const std::set<char> frontier_walkable{'.', 'G'};
    for (char terrain : kMovingAiTerrain) {
        tests.expect(
            verifier_accepts(terrain) == (verifier_walkable.count(terrain) != 0),
            std::string("unexpected verifier characterization for ") + terrain);
        tests.expect(
            mdd_accepts(terrain) == (mdd_walkable.count(terrain) != 0),
            std::string("unexpected MDD characterization for ") + terrain);

        const Grid one_cell{{terrain}};
        const auto seeded_zone =
            create_shape_from_conflicts({{0, 0}}, 0, one_cell);
        tests.expect(
            seeded_zone.count({0, 0}) == 1,
            std::string("current zone seeding should preserve in-bounds conflict cell ") +
                terrain);

        const auto frontier =
            find_new_positions({{0, 0}}, {}, one_cell);
        tests.expect(
            (frontier.count({0, 0}) != 0) ==
                (frontier_walkable.count(terrain) != 0),
            std::string("unexpected zone-frontier characterization for ") + terrain);
    }

    const auto goal_fallback = characterize_goal_only_fallback();
    tests.expect(
        goal_fallback.status == SolveStatus::InvalidInput,
        "current full-map counting treats a goal-only grid as having no walkable cells");

    // Row/column limits and invalid positions must be handled without indexing
    // outside the grid, even before all components agree on terrain.
    const Grid boundary_grid{{'.', '.'}, {'.', '.'}};
    MDDConstructor boundary_mdd(boundary_grid, {0, 0}, {1, 1}, 2);
    tests.expect(boundary_mdd.get_neighbors({0, 0}).size() == 2,
                 "top-left MDD neighbor count should respect row/column boundaries");
    tests.expect(boundary_mdd.get_neighbors({1, 1}).size() == 2,
                 "bottom-right MDD neighbor count should respect row/column boundaries");

    for (const Position& invalid :
         std::array<Position, 4>{{{-1, 0}, {2, 0}, {0, -1}, {0, 2}}}) {
        bool rejected_start = false;
        try {
            MDDConstructor invalid_mdd(boundary_grid, invalid, {0, 0}, 2);
        } catch (const std::invalid_argument&) {
            rejected_start = true;
        }
        tests.expect(rejected_start, "MDD should reject out-of-grid start positions");
        tests.expect(
            create_shape_from_conflicts({invalid}, 1, boundary_grid).empty(),
            "zone builder should reject out-of-grid conflict positions");
        tests.expect(
            has_issue(
                mapf::verify_path_geometry({invalid}, boundary_grid),
                mapf::VerificationIssueCode::OutOfBounds),
            "verifier should reject out-of-grid path positions");
    }

    return tests.finish();
}
