#pragma once

// Shared outcome vocabulary used at solver boundaries. The status describes a
// conclusion, while mutable path/waiting data remains in CurrentSolution.
enum class SolveStatus {
    Solved,
    Exhausted,
    InvalidInput,
    InvalidState
};

inline const char* solve_status_name(SolveStatus status) {
    switch (status) {
        case SolveStatus::Solved:
            return "solved";
        case SolveStatus::Exhausted:
            return "exhausted";
        case SolveStatus::InvalidInput:
            return "invalid_input";
        case SolveStatus::InvalidState:
            return "invalid_state";
    }
    return "invalid_state";
}

// Stable process exit codes let scripts distinguish an ordinary bounded-search
// failure from a bad invocation or an internal correctness failure.
inline int solve_status_exit_code(SolveStatus status) {
    switch (status) {
        case SolveStatus::Solved:
            return 0;
        case SolveStatus::Exhausted:
            return 1;
        case SolveStatus::InvalidInput:
            return 2;
        case SolveStatus::InvalidState:
            return 3;
    }
    return 3;
}
