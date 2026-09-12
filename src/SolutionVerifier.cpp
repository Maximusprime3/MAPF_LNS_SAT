#include "lnssat/SolutionVerifier.h"

#include <cstdlib>
#include <map>
#include <sstream>

namespace mapf {
namespace {

void add_issue(
    VerificationReport& report,
    VerificationIssueCode code,
    const std::string& message,
    int agent_id = -1,
    int timestep = -1,
    int other_agent_id = -1) {
    report.issues.push_back({code, message, agent_id, other_agent_id, timestep});
}

bool validate_grid(const Grid& grid, VerificationReport& report) {
    // Validate the shape once before any code indexes grid.front() or a cell.
    if (grid.empty() || grid.front().empty()) {
        add_issue(report, VerificationIssueCode::EmptyMap, "map must contain at least one cell");
        return false;
    }

    const std::size_t width = grid.front().size();
    for (std::size_t row = 0; row < grid.size(); ++row) {
        if (grid[row].size() != width) {
            std::ostringstream message;
            message << "map row " << row << " has width " << grid[row].size()
                    << ", expected " << width;
            add_issue(report, VerificationIssueCode::NonRectangularMap, message.str());
            return false;
        }
    }
    return true;
}

void validate_path_geometry(
    const Path& path,
    const Grid& grid,
    int agent_id,
    VerificationReport& report) {
    if (path.empty()) {
        std::ostringstream message;
        message << "agent " << agent_id << " has an empty path";
        add_issue(report, VerificationIssueCode::EmptyPath, message.str(), agent_id);
        return;
    }

    const int rows = static_cast<int>(grid.size());
    const int columns = static_cast<int>(grid.front().size());

    for (std::size_t t = 0; t < path.size(); ++t) {
        const Position position = path[t];
        if (position.first < 0 || position.first >= rows ||
            position.second < 0 || position.second >= columns) {
            std::ostringstream message;
            message << "agent " << agent_id << " is out of bounds at timestep " << t
                    << " at (" << position.first << ',' << position.second << ')';
            add_issue(report, VerificationIssueCode::OutOfBounds, message.str(), agent_id,
                      static_cast<int>(t));
        } else if (!is_walkable_cell(grid[position.first][position.second])) {
            std::ostringstream message;
            message << "agent " << agent_id << " occupies blocked cell ("
                    << position.first << ',' << position.second << ") at timestep " << t;
            add_issue(report, VerificationIssueCode::BlockedCell, message.str(), agent_id,
                      static_cast<int>(t));
        }

        if (t == 0) {
            continue;
        }

        const Position previous = path[t - 1];
        const long long distance = std::llabs(static_cast<long long>(position.first) - previous.first) +
                                   std::llabs(static_cast<long long>(position.second) - previous.second);
        if (distance > 1) {
            std::ostringstream message;
            message << "agent " << agent_id << " makes an illegal move from ("
                    << previous.first << ',' << previous.second << ") to ("
                    << position.first << ',' << position.second << ") at timestep " << t;
            add_issue(report, VerificationIssueCode::IllegalMove, message.str(), agent_id,
                      static_cast<int>(t));
        }
    }
}

}  // namespace

VerificationReport verify_path_geometry(
    const Path& path,
    const Grid& grid,
    int agent_id) {
    VerificationReport report;
    if (!validate_grid(grid, report)) {
        return report;
    }

    validate_path_geometry(path, grid, agent_id, report);
    if (!path.empty()) {
        report.makespan = static_cast<int>(path.size()) - 1;
    }
    return report;
}

VerificationReport verify_solution(
    const AgentPaths& agent_paths,
    const std::vector<Position>& starts,
    const std::vector<Position>& goals,
    const Grid& grid) {
    VerificationReport report;
    if (!validate_grid(grid, report)) {
        return report;
    }

    if (starts.size() != goals.size()) {
        std::ostringstream message;
        message << "received " << starts.size() << " starts but " << goals.size() << " goals";
        add_issue(report, VerificationIssueCode::StartGoalCountMismatch, message.str());
        return report;
    }

    const std::size_t expected_agents = starts.size();
    // The path container cannot hold duplicate keys, so checking the expected
    // ID range plus the missing-ID loop below proves exact agent coverage.
    for (const auto& entry : agent_paths) {
        const int agent_id = entry.first;
        if (agent_id < 0 || static_cast<std::size_t>(agent_id) >= expected_agents) {
            std::ostringstream message;
            message << "solution contains unexpected agent ID " << agent_id;
            add_issue(report, VerificationIssueCode::UnexpectedAgent, message.str(), agent_id);
        }
    }

    bool can_check_collisions = true;
    std::size_t common_path_length = 0;
    bool have_common_path_length = false;

    for (std::size_t expected_id = 0; expected_id < expected_agents; ++expected_id) {
        const int agent_id = static_cast<int>(expected_id);
        const auto path_it = agent_paths.find(agent_id);
        if (path_it == agent_paths.end()) {
            std::ostringstream message;
            message << "solution is missing expected agent " << agent_id;
            add_issue(report, VerificationIssueCode::MissingAgent, message.str(), agent_id);
            can_check_collisions = false;
            continue;
        }

        const Path& path = path_it->second;
        if (path.empty()) {
            validate_path_geometry(path, grid, agent_id, report);
            can_check_collisions = false;
            continue;
        }

        if (!have_common_path_length) {
            common_path_length = path.size();
            report.makespan = static_cast<int>(common_path_length) - 1;
            have_common_path_length = true;
        } else if (path.size() != common_path_length) {
            std::ostringstream message;
            message << "agent " << agent_id << " has path length " << path.size()
                    << ", expected " << common_path_length;
            add_issue(report, VerificationIssueCode::UnequalPathLength, message.str(), agent_id);
            can_check_collisions = false;
        }

        if (path.front() != starts[expected_id]) {
            std::ostringstream message;
            message << "agent " << agent_id << " starts at (" << path.front().first << ','
                    << path.front().second << ") instead of (" << starts[expected_id].first
                    << ',' << starts[expected_id].second << ')';
            add_issue(report, VerificationIssueCode::StartMismatch, message.str(), agent_id, 0);
        }

        if (path.back() != goals[expected_id]) {
            std::ostringstream message;
            message << "agent " << agent_id << " ends at (" << path.back().first << ','
                    << path.back().second << ") instead of (" << goals[expected_id].first
                    << ',' << goals[expected_id].second << ')';
            add_issue(report, VerificationIssueCode::GoalMismatch, message.str(), agent_id,
                      static_cast<int>(path.size()) - 1);
        }

        validate_path_geometry(path, grid, agent_id, report);
    }

    // Collision checks index all agents at a shared timestep. Skip them when a
    // path is missing, empty, or has a different length; those structural
    // errors have already been reported without risking invalid indexing.
    if (!can_check_collisions || !have_common_path_length) {
        return report;
    }

    // A position can have only one owner at each timestep. Every failed map
    // insertion identifies a vertex conflict with the first occupying agent.
    for (std::size_t t = 0; t < common_path_length; ++t) {
        std::map<Position, int> occupied;
        for (std::size_t expected_id = 0; expected_id < expected_agents; ++expected_id) {
            const int agent_id = static_cast<int>(expected_id);
            const Position position = agent_paths.at(agent_id)[t];
            const auto inserted = occupied.emplace(position, agent_id);
            if (!inserted.second) {
                std::ostringstream message;
                message << "agents " << inserted.first->second << " and " << agent_id
                        << " occupy (" << position.first << ',' << position.second
                        << ") at timestep " << t;
                add_issue(report, VerificationIssueCode::VertexConflict, message.str(),
                          inserted.first->second, static_cast<int>(t), agent_id);
            }
        }
    }

    // Check unordered agent pairs for opposite traversal of the same edge.
    // Requiring a real move avoids classifying two stationary agents on an
    // already-conflicting vertex as an edge swap as well.
    for (std::size_t t = 1; t < common_path_length; ++t) {
        for (std::size_t first = 0; first < expected_agents; ++first) {
            for (std::size_t second = first + 1; second < expected_agents; ++second) {
                const Path& first_path = agent_paths.at(static_cast<int>(first));
                const Path& second_path = agent_paths.at(static_cast<int>(second));
                if (first_path[t - 1] != first_path[t] &&
                    first_path[t - 1] == second_path[t] &&
                    first_path[t] == second_path[t - 1]) {
                    std::ostringstream message;
                    message << "agents " << first << " and " << second
                            << " swap edges at timestep " << t;
                    add_issue(report, VerificationIssueCode::EdgeConflict, message.str(),
                              static_cast<int>(first), static_cast<int>(t),
                              static_cast<int>(second));
                }
            }
        }
    }

    return report;
}

const char* verification_issue_code_name(VerificationIssueCode code) {
    switch (code) {
        case VerificationIssueCode::EmptyMap: return "empty_map";
        case VerificationIssueCode::NonRectangularMap: return "non_rectangular_map";
        case VerificationIssueCode::StartGoalCountMismatch: return "start_goal_count_mismatch";
        case VerificationIssueCode::MissingAgent: return "missing_agent";
        case VerificationIssueCode::UnexpectedAgent: return "unexpected_agent";
        case VerificationIssueCode::EmptyPath: return "empty_path";
        case VerificationIssueCode::UnequalPathLength: return "unequal_path_length";
        case VerificationIssueCode::StartMismatch: return "start_mismatch";
        case VerificationIssueCode::GoalMismatch: return "goal_mismatch";
        case VerificationIssueCode::OutOfBounds: return "out_of_bounds";
        case VerificationIssueCode::BlockedCell: return "blocked_cell";
        case VerificationIssueCode::IllegalMove: return "illegal_move";
        case VerificationIssueCode::VertexConflict: return "vertex_conflict";
        case VerificationIssueCode::EdgeConflict: return "edge_conflict";
    }
    return "unknown";
}

}  // namespace mapf
