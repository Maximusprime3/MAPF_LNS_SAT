#pragma once

#include "NeighborhoodVariant.h"
#include "SolverConfiguration.h"
#include "SolveStatus.h"

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
    SatBackend backend = SatBackend::MiniSat;
    bool search_started = false;
    std::string message;

    bool solved() const {
        return status == SolveStatus::Solved;
    }
};

LNSResult LNS(
    const SolveRequest& request,
    const SolverConfig& config);

LNSResult LNS(
    const std::string& map_path,
    const std::string& scenario_path,
    int num_agents,
    int scenario_index,
    bool use_minisat,
    int seed,
    NeighborhoodVariant variant = NeighborhoodVariant::LnsSat);
