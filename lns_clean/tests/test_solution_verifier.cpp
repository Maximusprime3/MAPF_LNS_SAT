#include "../SolutionVerifier.h"

#include <initializer_list>
#include <iostream>
#include <string>

namespace {

mapf::Grid make_grid(std::initializer_list<std::string> rows) {
    mapf::Grid grid;
    for (const auto& row : rows) {
        grid.emplace_back(row.begin(), row.end());
    }
    return grid;
}

bool contains_issue(
    const mapf::VerificationReport& report,
    mapf::VerificationIssueCode expected) {
    for (const auto& issue : report.issues) {
        if (issue.code == expected) {
            return true;
        }
    }
    return false;
}

class TestRunner {
public:
    // Keep the test executable dependency-free: each expectation records a
    // failure and the process returns non-zero after all cases have run.
    void expect_valid(const std::string& name, const mapf::VerificationReport& report) {
        ++tests_run_;
        if (!report.valid()) {
            ++failures_;
            std::cerr << "FAIL " << name << ": expected valid, got" << std::endl;
            print_issues(report);
        }
    }

    void expect_issue(
        const std::string& name,
        const mapf::VerificationReport& report,
        mapf::VerificationIssueCode expected) {
        ++tests_run_;
        if (report.valid() || !contains_issue(report, expected)) {
            ++failures_;
            std::cerr << "FAIL " << name << ": expected issue "
                      << mapf::verification_issue_code_name(expected) << std::endl;
            print_issues(report);
        }
    }

    void expect_no_issue(
        const std::string& name,
        const mapf::VerificationReport& report,
        mapf::VerificationIssueCode unexpected) {
        ++tests_run_;
        if (contains_issue(report, unexpected)) {
            ++failures_;
            std::cerr << "FAIL " << name << ": unexpected issue "
                      << mapf::verification_issue_code_name(unexpected) << std::endl;
            print_issues(report);
        }
    }

    int finish() const {
        if (failures_ == 0) {
            std::cout << "PASS: " << tests_run_ << " deterministic verifier tests" << std::endl;
            return 0;
        }
        std::cerr << "FAIL: " << failures_ << " of " << tests_run_ << " tests failed" << std::endl;
        return 1;
    }

private:
    static void print_issues(const mapf::VerificationReport& report) {
        if (report.issues.empty()) {
            std::cerr << "  no issues were reported" << std::endl;
            return;
        }
        for (const auto& issue : report.issues) {
            std::cerr << "  [" << mapf::verification_issue_code_name(issue.code)
                      << "] " << issue.message << std::endl;
        }
    }

    int tests_run_ = 0;
    int failures_ = 0;
};

}  // namespace

int main() {
    using mapf::AgentPaths;
    using mapf::Position;
    using mapf::VerificationIssueCode;

    TestRunner tests;
    const auto open_grid = make_grid({"....", ".@..", "...."});

    // A small two-agent baseline used by several structural test cases.
    const std::vector<Position> valid_starts{{0, 0}, {2, 3}};
    const std::vector<Position> valid_goals{{0, 3}, {2, 0}};
    const AgentPaths valid_paths{
        {0, {{0, 0}, {0, 1}, {0, 2}, {0, 3}}},
        {1, {{2, 3}, {2, 2}, {2, 1}, {2, 0}}}
    };
    tests.expect_valid(
        "valid collision-free solution",
        mapf::verify_solution(valid_paths, valid_starts, valid_goals, open_grid));

    tests.expect_valid(
        "wait moves and goal terrain",
        mapf::verify_solution(
            AgentPaths{{0, {{0, 0}, {0, 0}, {0, 1}, {0, 2}}}},
            {{0, 0}},
            {{0, 2}},
            make_grid({".G."})));

    // Agent-set and path-shape failures.
    tests.expect_issue(
        "missing expected agent",
        mapf::verify_solution(
            AgentPaths{{0, valid_paths.at(0)}}, valid_starts, valid_goals, open_grid),
        VerificationIssueCode::MissingAgent);

    AgentPaths unexpected_paths{{0, {{0, 0}}}, {7, {{2, 3}}}};
    tests.expect_issue(
        "unexpected agent ID",
        mapf::verify_solution(unexpected_paths, {{0, 0}}, {{0, 0}}, open_grid),
        VerificationIssueCode::UnexpectedAgent);

    tests.expect_issue(
        "empty path",
        mapf::verify_solution(AgentPaths{{0, {}}}, {{0, 0}}, {{0, 0}}, open_grid),
        VerificationIssueCode::EmptyPath);

    tests.expect_issue(
        "unequal path lengths",
        mapf::verify_solution(
            AgentPaths{
                {0, valid_paths.at(0)},
                {1, {{2, 3}, {2, 2}, {2, 1}, {2, 0}, {2, 0}}}},
            valid_starts,
            valid_goals,
            open_grid),
        VerificationIssueCode::UnequalPathLength);

    // Per-agent geometry and endpoint failures.
    tests.expect_issue(
        "wrong start",
        mapf::verify_solution(
            AgentPaths{{0, {{0, 1}, {0, 2}, {0, 3}}}},
            {{0, 0}},
            {{0, 3}},
            open_grid),
        VerificationIssueCode::StartMismatch);

    tests.expect_issue(
        "wrong goal",
        mapf::verify_solution(
            AgentPaths{{0, {{0, 0}, {0, 1}, {0, 2}}}},
            {{0, 0}},
            {{0, 3}},
            open_grid),
        VerificationIssueCode::GoalMismatch);

    tests.expect_issue(
        "out of bounds",
        mapf::verify_solution(
            AgentPaths{{0, {{0, 0}, {-1, 0}}}},
            {{0, 0}},
            {{-1, 0}},
            open_grid),
        VerificationIssueCode::OutOfBounds);

    tests.expect_issue(
        "blocked cell",
        mapf::verify_solution(
            AgentPaths{{0, {{1, 0}, {1, 1}}}},
            {{1, 0}},
            {{1, 1}},
            open_grid),
        VerificationIssueCode::BlockedCell);

    tests.expect_issue(
        "illegal move",
        mapf::verify_solution(
            AgentPaths{{0, {{0, 0}, {0, 2}}}},
            {{0, 0}},
            {{0, 2}},
            open_grid),
        VerificationIssueCode::IllegalMove);

    // Multi-agent collision failures. The shared-wait regression ensures that
    // vertex occupancy and edge swaps remain distinct error categories.
    tests.expect_issue(
        "vertex conflict",
        mapf::verify_solution(
            AgentPaths{
                {0, {{0, 0}, {0, 1}}},
                {1, {{0, 2}, {0, 1}}}},
            {{0, 0}, {0, 2}},
            {{0, 1}, {0, 1}},
            make_grid({"..."})),
        VerificationIssueCode::VertexConflict);

    const auto shared_wait_report = mapf::verify_solution(
        AgentPaths{
            {0, {{0, 0}, {0, 0}}},
            {1, {{0, 0}, {0, 0}}}},
        {{0, 0}, {0, 0}},
        {{0, 0}, {0, 0}},
        make_grid({"."}));
    tests.expect_issue(
        "shared wait is a vertex conflict",
        shared_wait_report,
        VerificationIssueCode::VertexConflict);
    tests.expect_no_issue(
        "shared wait is not an edge swap",
        shared_wait_report,
        VerificationIssueCode::EdgeConflict);

    tests.expect_issue(
        "edge-swap conflict",
        mapf::verify_solution(
            AgentPaths{
                {0, {{0, 0}, {0, 1}}},
                {1, {{0, 1}, {0, 0}}}},
            {{0, 0}, {0, 1}},
            {{0, 1}, {0, 0}},
            make_grid({".."})),
        VerificationIssueCode::EdgeConflict);

    // Invalid problem definitions should be rejected without indexing them.
    tests.expect_issue(
        "non-rectangular map",
        mapf::verify_path_geometry({{0, 0}}, make_grid({"..", "."})),
        VerificationIssueCode::NonRectangularMap);

    tests.expect_issue(
        "empty map",
        mapf::verify_path_geometry({{0, 0}}, {}),
        VerificationIssueCode::EmptyMap);

    tests.expect_issue(
        "start/goal count mismatch",
        mapf::verify_solution(AgentPaths{}, {{0, 0}}, {}, open_grid),
        VerificationIssueCode::StartGoalCountMismatch);

    return tests.finish();
}
