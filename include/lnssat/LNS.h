#pragma once

#include "lnssat/NeighborhoodVariant.h"
#include "lnssat/SolverConfiguration.h"
#include "lnssat/SolveStatus.h"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using AgentPaths = std::unordered_map<int, std::vector<std::pair<int, int>>>;

// Public result of one bounded LNS-SAT invocation. Paths are populated only
// for Solved; every other outcome carries a diagnostic instead of relying on
// an ambiguous empty path map.
struct LNSResult {
    SolveStatus status = SolveStatus::Exhausted;
    AgentPaths paths;
    int makespan = -1;
    double runtime_ms = 0.0;
    int seed = 0;
    NeighborhoodVariant neighborhood_variant = NeighborhoodVariant::LnsSat;
    bool search_started = false;
    std::string message;

    bool solved() const {
        return status == SolveStatus::Solved;
    }
};

LNSResult LNS(
    const SolveRequest& request,
    const SolverConfig& config);
