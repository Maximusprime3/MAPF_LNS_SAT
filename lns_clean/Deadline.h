#pragma once

#include <chrono>
#include <optional>

using SolverDeadline = std::optional<std::chrono::steady_clock::time_point>;

inline bool solver_deadline_reached(const SolverDeadline& deadline) {
    return deadline && std::chrono::steady_clock::now() >= *deadline;
}
