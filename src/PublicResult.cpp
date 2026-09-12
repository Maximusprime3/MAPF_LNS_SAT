#include "lnssat/PublicResult.h"
#include <climits>
#include <set>
#include <stdexcept>

namespace lnssat {
namespace {
const Json& field(const Json& j, const char* name) {
    if (!j.is_object() || !j.as_object().count(name)) throw std::invalid_argument(std::string("Missing field: ") + name);
    return j.as_object().at(name);
}
void keys(const Json& j, std::initializer_list<const char*> names) {
    if (!j.is_object() || j.as_object().size() != names.size()) throw std::invalid_argument("Unexpected or missing object fields");
    for (auto name : names) (void)field(j,name);
}
std::string str(const Json& j) {
    if (!j.is_string()) throw std::invalid_argument("Expected string");
    return j.as_string();
}
int integer(const Json& j, int minimum = 0) {
    if (!j.is_integer() || j.as_integer() < minimum || j.as_integer() > INT_MAX) throw std::invalid_argument("Expected integer in range");
    return static_cast<int>(j.as_integer());
}
void number(const Json& j) {
    if (!j.is_number() || !std::isfinite(j.as_number()) || j.as_number() < 0) throw std::invalid_argument("Expected finite nonnegative timing");
}
std::string hash(const Json& j, bool nullable = false) {
    if (nullable && j.is_null()) return {};
    const auto value = str(j);
    if (value.size() != 64 || value.find_first_not_of("0123456789abcdef") != std::string::npos) throw std::invalid_argument("Expected SHA-256 hex string");
    return value;
}
void nullable_string(const Json& j) { if (!j.is_null()) (void)str(j); }
Json optional_int(const std::optional<int>& value) { return value ? Json(*value) : Json(); }
}

Json result_document(const SolveRequest& request, const SolverConfig& config,
                     const LNSResult& result, const InputSnapshot& inputs,
                     double total_ms, Json solver_ms, const Json& build) {
    const int64_t first = static_cast<int64_t>(request.scenario_index) * request.num_agents;
    Object instance{{"map_path",request.map_path},{"scenario_path",request.scenario_path},
        {"map_sha256",inputs.map_hash},{"scenario_sha256",inputs.scenario_hash},
        {"agents",request.num_agents},{"scenario_index",request.scenario_index},
        {"first_row",first},{"last_row_exclusive",first + request.num_agents}};
    Object configuration{{"seed",config.seed},{"variant",neighborhood_variant_name(config.neighborhood_variant)},
        {"makespan_increment",config.makespan_increment},{"makespan_increase_limit",config.makespan_increase_limit},
        {"makespan_bound",optional_int(config.makespan_bound)},
        {"lazy_iteration_limit",config.lazy_iteration_limit},{"full_map_fallback_threshold",config.full_map_fallback_threshold},
        {"wall_clock_limit_ms",config.wall_clock_limit ? Json(static_cast<int64_t>(config.wall_clock_limit->count())) : Json()},
        {"log_level",log_level_name(config.log_level)},
        {"log",config.log_output_path.empty() ? Json() : Json(config.log_output_path)}};
    Json solution;
    if (result.solved()) {
        Array agents;
        for (int id = 0; id < request.num_agents; ++id) {
            Array positions;
            for (const auto& p : result.paths.at(id)) positions.emplace_back(Array{p.first,p.second});
            agents.emplace_back(Object{{"id",id},{"positions",std::move(positions)}});
        }
        solution = Object{{"coordinate_order","row,column"},{"horizon",result.makespan + 1},{"agents",std::move(agents)}};
    }
    return Object{{"schema_version",1},{"status",solve_status_name(result.status)},
        {"diagnostic",result.message},{"termination_reason",result.termination_reason},
        {"instance",std::move(instance)},{"configuration",std::move(configuration)},
        {"runtime",Object{{"total_ms",total_ms},{"solver_ms",solver_ms}}},
        {"makespan",result.solved() ? Json(result.makespan) : Json()},
        {"verification",result.verification},{"build",build},{"solution",solution}};
}

SubmittedSolution read_result_document(const Json& document) {
    keys(document,{"schema_version","status","diagnostic","termination_reason","instance","configuration","runtime","makespan","verification","build","solution"});
    if (integer(field(document,"schema_version")) != 1) throw std::invalid_argument("Unsupported schema_version");
    const std::string status = str(field(document,"status"));
    if (status != "solved" && status != "exhausted" && status != "invalid_input" && status != "invalid_state") throw std::invalid_argument("Unknown solve status");
    (void)str(field(document,"diagnostic"));
    const auto reason = str(field(document,"termination_reason"));
    const std::set<std::string> reasons{"solved","search_exhausted","makespan_bound","wall_clock_limit","invalid_input","internal_failure"};
    if (!reasons.count(reason)) throw std::invalid_argument("Unknown termination reason");
    const auto verification = str(field(document,"verification"));
    if (verification != "passed" && verification != "failed" && verification != "not_performed") throw std::invalid_argument("Unknown verification state");
    const auto& runtime = field(document,"runtime");
    keys(runtime,{"total_ms","solver_ms"}); number(field(runtime,"total_ms"));
    if (!field(runtime,"solver_ms").is_null()) number(field(runtime,"solver_ms"));
    const auto& build = field(document,"build");
    keys(build,{"revision","dirty","source_sha256","minisat","compiler","configuration","platform"});
    nullable_string(field(build,"revision"));
    if (!field(build,"dirty").is_null() && !field(build,"dirty").is_bool()) throw std::invalid_argument("Invalid build dirty state");
    (void)hash(field(build,"source_sha256"),true);
    for (auto key : {"compiler","configuration","platform"}) nullable_string(field(build,key));
    const auto& backend = field(build,"minisat");
    keys(backend,{"identity","version","source_sha256"});
    (void)str(field(backend,"identity")); nullable_string(field(backend,"version")); (void)hash(field(backend,"source_sha256"),true);
    SubmittedSolution submitted;
    const auto& instance = field(document,"instance");
    keys(instance,{"map_path","scenario_path","map_sha256","scenario_sha256","agents","scenario_index","first_row","last_row_exclusive"});
    auto& request = submitted.request;
    request.map_path = str(field(instance,"map_path")); request.scenario_path = str(field(instance,"scenario_path"));
    request.num_agents = integer(field(instance,"agents"),1); request.scenario_index = integer(field(instance,"scenario_index"));
    submitted.map_hash = hash(field(instance,"map_sha256"),status != "solved");
    submitted.scenario_hash = hash(field(instance,"scenario_sha256"),status != "solved");
    const int64_t first = static_cast<int64_t>(request.scenario_index) * request.num_agents;
    for (auto key : {"first_row","last_row_exclusive"}) {
        if (!field(instance,key).is_integer() || field(instance,key).as_integer() < 0) throw std::invalid_argument("Invalid row range");
    }
    submitted.selection_consistent = field(instance,"first_row").as_integer() == first && field(instance,"last_row_exclusive").as_integer() == first + request.num_agents;
    const auto& configuration = field(document,"configuration");
    keys(configuration,{"seed","variant","makespan_increment","makespan_increase_limit","makespan_bound","lazy_iteration_limit","full_map_fallback_threshold","wall_clock_limit_ms","log_level","log"});
    SolverConfig config;
    for (const auto& item : configuration.as_object()) {
        if (item.second.is_null()) {
            if (item.first != "makespan_bound" && item.first != "wall_clock_limit_ms" && item.first != "log") throw std::invalid_argument("Unexpected null configuration field");
            continue;
        }
        std::string value;
        if (item.first == "variant" || item.first == "log_level" || item.first == "log") value = str(item.second);
        else if (item.first == "full_map_fallback_threshold") {
            if (!item.second.is_number()) throw std::invalid_argument("Expected numeric threshold");
            value = lnssat_json::dump(item.second); value.pop_back();
        } else value = std::to_string(integer(item.second, item.first == "seed" ? INT_MIN : 0));
        if (auto error = apply_solver_configuration_value(item.first,value,request,config)) throw std::invalid_argument(*error);
    }
    auto validation = validate_solver_configuration(request,config);
    if (!validation.valid) throw std::invalid_argument(validation.message);
    if (str(field(configuration,"variant")) != neighborhood_variant_name(config.neighborhood_variant)) throw std::invalid_argument("Expected canonical variant");
    const auto& solution = field(document,"solution");
    if (status != "solved") {
        if (!solution.is_null() || !field(document,"makespan").is_null()) throw std::invalid_argument("Unsuccessful result must not contain a solution or makespan");
        return submitted;
    }
    submitted.has_solution = true;
    submitted.makespan = integer(field(document,"makespan"));
    keys(solution,{"coordinate_order","horizon","agents"});
    if (str(field(solution,"coordinate_order")) != "row,column") throw std::invalid_argument("Unsupported coordinate convention");
    const int horizon = integer(field(solution,"horizon"),1);
    const auto& agents = field(solution,"agents");
    if (!agents.is_array() || agents.as_array().size() != static_cast<size_t>(request.num_agents)) throw std::invalid_argument("Inconsistent agent count");
    for (const auto& agent : agents.as_array()) {
        keys(agent,{"id","positions"});
        int id = integer(field(agent,"id"));
        const auto& positions = field(agent,"positions");
        if (!positions.is_array() || positions.as_array().size() != static_cast<size_t>(horizon)) throw std::invalid_argument("Inconsistent path horizon");
        std::vector<std::pair<int,int>> path;
        for (const auto& p : positions.as_array()) {
            if (!p.is_array() || p.as_array().size() != 2) throw std::invalid_argument("Expected coordinate pair");
            path.emplace_back(integer(p.as_array()[0],INT_MIN),integer(p.as_array()[1],INT_MIN));
        }
        if (!submitted.paths.emplace(id,std::move(path)).second) throw std::invalid_argument("Duplicate agent ID");
    }
    return submitted;
}
}
