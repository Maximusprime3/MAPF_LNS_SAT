#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mapf {

using Position = std::pair<int, int>;
using Path = std::vector<Position>;
using Grid = std::vector<std::vector<char>>;
using AgentPaths = std::unordered_map<int, Path>;

// Stable machine-readable categories let callers react to verification
// failures without parsing human-readable log messages.
enum class VerificationIssueCode {
    EmptyMap,
    NonRectangularMap,
    StartGoalCountMismatch,
    MissingAgent,
    UnexpectedAgent,
    EmptyPath,
    UnequalPathLength,
    StartMismatch,
    GoalMismatch,
    OutOfBounds,
    BlockedCell,
    IllegalMove,
    VertexConflict,
    EdgeConflict
};

// Agent/timestep fields are populated when the issue can be localized. For a
// two-agent conflict, other_agent_id identifies the second participant.
struct VerificationIssue {
    VerificationIssueCode code;
    std::string message;
    int agent_id = -1;
    int other_agent_id = -1;
    int timestep = -1;
};

// A valid report has no issues. Makespan is the number of moves in each path,
// or -1 when no common non-empty path length could be established.
struct VerificationReport {
    std::vector<VerificationIssue> issues;
    int makespan = -1;

    bool valid() const {
        return issues.empty();
    }
};

// Moving AI maps use '.', 'G', and 'S' for traversable terrain.
bool is_walkable_cell(char cell);

// Validates one path's map bounds, terrain, and moves. It intentionally does
// not make start/goal or multi-agent claims.
VerificationReport verify_path_geometry(
    const Path& path,
    const Grid& grid,
    int agent_id = -1);

// Validates a complete, fixed-makespan MAPF solution. Expected agent IDs are
// exactly 0..starts.size()-1.
VerificationReport verify_solution(
    const AgentPaths& agent_paths,
    const std::vector<Position>& starts,
    const std::vector<Position>& goals,
    const Grid& grid);

const char* verification_issue_code_name(VerificationIssueCode code);

}  // namespace mapf
