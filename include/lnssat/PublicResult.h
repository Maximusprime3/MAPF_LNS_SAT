#pragma once
#include "lnssat/Json.h"
#include "lnssat/LNS.h"
#include "lnssat/Load_LNSProblem.h"

namespace lnssat {
using Json = lnssat_json::JsonValue;
using Object = lnssat_json::JsonObject;
using Array = lnssat_json::JsonArray;
struct InputSnapshot {
    std::string map_bytes, scenario_bytes;
    Json map_hash, scenario_hash;
};
Json result_document(const SolveRequest&, const SolverConfig&, const LNSResult&,
                     const InputSnapshot&, double total_ms, Json solver_ms,
                     const Json& build);
struct SubmittedSolution {
    SolveRequest request;
    AgentPaths paths;
    int makespan = -1;
    std::string map_hash, scenario_hash;
    bool has_solution = false;
    bool selection_consistent = true;
};
// Throws invalid_argument for malformed schema. Semantic mismatches remain
// independently verifiable and are reported as invalid submissions, not solver errors.
SubmittedSolution read_result_document(const Json& document);
using SolveFunction = LNSResult (*)(const SolveRequest&, const SolverConfig&,
                                    const LNSProblem&, SolverDeadline);
int public_cli(int argc, char** argv, SolveFunction solve);
}
