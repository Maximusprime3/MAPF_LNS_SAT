#include "lnssat/ExperimentLogger.h"

#include <chrono>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <stdexcept>

namespace {
constexpr int kFloatPrecision = 6;
void finish_file(std::ofstream& file, const std::filesystem::path& path) {
    file.flush();
    if (!file) throw std::runtime_error("Failed writing metrics: " + path.string());
    file.close();
    if (!file) throw std::runtime_error("Failed closing metrics: " + path.string());
}

constexpr const char* kLazyIterationsHeader =
    "experiment_id,makespan_attempt,zone_attempt,waiting_attempt,iteration,clauses_before,variables,clauses_added,total_clauses_after," \
    "new_vertex_collisions,new_edge_collisions,total_vertex_collisions,total_edge_collisions,iteration_wall_ms,solver_wall_ms,solver_reported_ms," \
    "solver_calls,used_assumptions,reset_solver,satisfiable,num_decisions,num_propagations";

constexpr const char* kWaitingAttemptsHeader =
    "experiment_id,makespan_attempt,zone_attempt,waiting_attempt,waiting_budget,zone_positions,segment_count,agent_count,start_t,end_t," \
    "cnf_clauses,cnf_variables,mdd_build_ms,cnf_build_ms,attempt_wall_ms,lazy_iterations,lazy_total_wall_ms,lazy_solver_wall_ms,lazy_solver_reported_ms," \
    "applied_waiting,extended_window,solved";

constexpr const char* kLocalZoneHeader =
    "experiment_id,makespan_attempt,zone_attempt,zone_positions,zone_fraction,conflicts,agents,start_t,end_t,waiting_attempts," \
    "total_lazy_iterations,total_cnf_clauses,total_cnf_variables,total_mdd_build_ms,total_cnf_build_ms,total_lazy_wall_ms,total_lazy_solver_wall_ms,total_lazy_solver_reported_ms,solved";

constexpr const char* kMakespanHeader =
    "experiment_id,makespan_attempt,makespan,zones_attempted,zones_solved,total_waiting_attempts,total_lazy_iterations,total_cnf_clauses,total_cnf_variables," \
    "total_mdd_build_ms,total_cnf_build_ms,total_lazy_wall_ms,total_lazy_solver_wall_ms,total_lazy_solver_reported_ms,attempt_wall_ms,solved";

constexpr const char* kExperimentHeader =
    "experiment_id,map_path,scenario_path,num_agents,scenario_index,seed,solved,makespan_success,total_runtime_ms,total_cnf_clauses,total_cnf_variables," \
    "total_mdd_build_ms,total_cnf_build_ms,total_lazy_wall_ms,total_lazy_solver_wall_ms,total_lazy_solver_reported_ms";
}

ExperimentLogger& ExperimentLogger::instance() {
    static ExperimentLogger instance;
    return instance;
}

ExperimentLogger::ExperimentLogger() = default;

void ExperimentLogger::set_output_directory(const std::filesystem::path& directory) {
    base_dir_ = directory;
    header_written_.clear();
}

void ExperimentLogger::set_log_level(LogLevel log_level) {
    log_level_ = log_level;
}

void ExperimentLogger::ensure_directory() const {
    std::error_code ec;
    std::filesystem::create_directories(base_dir_, ec);
    if (ec) throw std::runtime_error("Cannot create metrics directory: " + ec.message());
}

std::filesystem::path ExperimentLogger::file_path(const std::string& filename) const {
    return base_dir_ / filename;
}

std::string ExperimentLogger::sanitize(const std::string& value) const {
    std::string sanitized;
    sanitized.reserve(value.size());
    for (char ch : value) {
        if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '-') {
            sanitized.push_back(ch);
        } else {
            sanitized.push_back('_');
        }
    }
    return sanitized;
}

void ExperimentLogger::ensure_header(const std::filesystem::path& path, const std::string& header) {
    ensure_directory();
    const auto key = path.string();
    auto it = header_written_.find(key);
    if (it != header_written_.end() && it->second) {
        return;
    }
    bool need_header = false;
    if (!std::filesystem::exists(path)) {
        need_header = true;
    } else if (std::filesystem::file_size(path) == 0) {
        need_header = true;
    }
    if (need_header) {
        std::ofstream file(path, std::ios::app);
        if (!file) throw std::runtime_error("Cannot open metrics: " + path.string());
        file << header << '\n';
        finish_file(file, path);
    }
    header_written_[key] = true;
}

std::string ExperimentLogger::start_experiment(const std::string& map_path,
                                               const std::string& scenario_path,
                                               int num_agents,
                                               int scenario_index,
                                               int seed) {
    if (base_dir_.empty() || log_level_ == LogLevel::Quiet) {
        return "logging-disabled";
    }
    ensure_directory();
    auto now = std::chrono::system_clock::now();
    auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    std::ostringstream oss;
    oss << sanitize(map_path) << "-" << sanitize(scenario_path)
        << "-agents" << num_agents
        << "-idx" << scenario_index
        << "-seed" << seed
        << "-" << ts;
    return oss.str();
}

void ExperimentLogger::log_lazy_iteration(const std::string& experiment_id,
                                          int makespan_attempt,
                                          int zone_attempt,
                                          int waiting_attempt,
                                          const LazySatIterationMetrics& metrics) {
    if (base_dir_.empty() || log_level_ == LogLevel::Quiet) return;
    auto path = file_path("lazy_iterations.csv");
    ensure_header(path, kLazyIterationsHeader);

    std::ofstream file(path, std::ios::app);
    if (!file.is_open()) throw std::runtime_error("Cannot open metrics: " + path.string());
    file << std::fixed << std::setprecision(kFloatPrecision);
    file << experiment_id << ',' << makespan_attempt << ',' << zone_attempt << ',' << waiting_attempt
         << ',' << metrics.iteration
         << ',' << metrics.clause_count_before
         << ',' << metrics.variable_count
         << ',' << metrics.clauses_added
         << ',' << metrics.total_clauses_after
         << ',' << metrics.new_vertex_collisions
         << ',' << metrics.new_edge_collisions
         << ',' << metrics.total_vertex_collisions
         << ',' << metrics.total_edge_collisions
         << ',' << metrics.iteration_wall_time_ms
         << ',' << metrics.solver_wall_time_ms
         << ',' << metrics.solver_reported_time_ms
         << ',' << metrics.solver_calls
         << ',' << (metrics.used_assumptions ? 1 : 0)
         << ',' << (metrics.reset_solver ? 1 : 0)
         << ',' << (metrics.satisfiable ? 1 : 0)
         << ',' << metrics.num_decisions
         << ',' << metrics.num_propagations
         << '\n';
    finish_file(file, path);
}

void ExperimentLogger::log_waiting_attempt(const std::string& experiment_id,
                                           int makespan_attempt,
                                           int zone_attempt,
                                           const WaitingAttemptMetrics& metrics) {
    if (base_dir_.empty() || log_level_ == LogLevel::Quiet) return;
    auto path = file_path("waiting_attempts.csv");
    ensure_header(path, kWaitingAttemptsHeader);
    std::ofstream file(path, std::ios::app);
    if (!file.is_open()) throw std::runtime_error("Cannot open metrics: " + path.string());
    const auto& lazy = metrics.lazy_metrics;
    file << std::fixed << std::setprecision(kFloatPrecision);
    file << experiment_id << ',' << makespan_attempt << ',' << zone_attempt << ',' << metrics.attempt_index
         << ',' << metrics.waiting_time_budget
         << ',' << metrics.zone_positions
         << ',' << metrics.segment_count
         << ',' << metrics.agent_count
         << ',' << metrics.start_t
         << ',' << metrics.end_t
         << ',' << metrics.cnf_clauses
         << ',' << metrics.cnf_variables
         << ',' << metrics.mdd_build_time_ms
         << ',' << metrics.cnf_build_time_ms
         << ',' << metrics.attempt_wall_time_ms
         << ',' << lazy.iterations.size()
         << ',' << lazy.total_wall_time_ms
         << ',' << lazy.total_solver_wall_time_ms
         << ',' << lazy.total_solver_reported_time_ms
         << ',' << (metrics.applied_waiting_time ? 1 : 0)
         << ',' << (metrics.extended_time_window ? 1 : 0)
         << ',' << (metrics.solved ? 1 : 0)
         << '\n';
    finish_file(file, path);
}

void ExperimentLogger::log_local_zone_attempt(const std::string& experiment_id,
                                              int makespan_attempt,
                                              const LocalZoneAttemptMetrics& metrics) {
    if (base_dir_.empty() || log_level_ == LogLevel::Quiet) return;
    auto path = file_path("local_zones.csv");
    ensure_header(path, kLocalZoneHeader);
    std::ofstream file(path, std::ios::app);
    if (!file.is_open()) throw std::runtime_error("Cannot open metrics: " + path.string());
    file << std::fixed << std::setprecision(kFloatPrecision);
    file << experiment_id << ',' << makespan_attempt << ',' << metrics.attempt_index
         << ',' << metrics.zone_positions
         << ',' << metrics.zone_fraction
         << ',' << metrics.conflicts
         << ',' << metrics.agents_in_window
         << ',' << metrics.start_t
         << ',' << metrics.end_t
         << ',' << metrics.waiting_attempts
         << ',' << metrics.total_lazy_iterations
         << ',' << metrics.total_cnf_clauses
         << ',' << metrics.total_cnf_variables
         << ',' << metrics.total_mdd_build_time_ms
         << ',' << metrics.total_cnf_build_time_ms
         << ',' << metrics.total_lazy_wall_time_ms
         << ',' << metrics.total_lazy_solver_wall_time_ms
         << ',' << metrics.total_lazy_solver_reported_time_ms
         << ',' << (metrics.solved ? 1 : 0)
         << '\n';
    finish_file(file, path);
}

void ExperimentLogger::log_makespan_attempt(const std::string& experiment_id,
                                            const MakespanAttemptMetrics& metrics) {
    if (base_dir_.empty() || log_level_ == LogLevel::Quiet) return;
    auto path = file_path("makespan_attempts.csv");
    ensure_header(path, kMakespanHeader);
    std::ofstream file(path, std::ios::app);
    if (!file.is_open()) throw std::runtime_error("Cannot open metrics: " + path.string());
    file << std::fixed << std::setprecision(kFloatPrecision);
    file << experiment_id << ',' << metrics.attempt_index
         << ',' << metrics.makespan
         << ',' << metrics.zones_attempted
         << ',' << metrics.zones_solved
         << ',' << metrics.total_waiting_attempts
         << ',' << metrics.total_lazy_iterations
         << ',' << metrics.total_cnf_clauses
         << ',' << metrics.total_cnf_variables
         << ',' << metrics.total_mdd_build_time_ms
         << ',' << metrics.total_cnf_build_time_ms
         << ',' << metrics.total_lazy_wall_time_ms
         << ',' << metrics.total_lazy_solver_wall_time_ms
         << ',' << metrics.total_lazy_solver_reported_time_ms
         << ',' << metrics.attempt_wall_time_ms
         << ',' << (metrics.solved ? 1 : 0)
         << '\n';
    finish_file(file, path);
}

void ExperimentLogger::log_experiment_summary(const ExperimentSummaryMetrics& metrics) {
    if (base_dir_.empty() || log_level_ == LogLevel::Quiet) return;
    auto path = file_path("experiments.csv");
    ensure_header(path, kExperimentHeader);
    std::ofstream file(path, std::ios::app);
    if (!file.is_open()) throw std::runtime_error("Cannot open metrics: " + path.string());
    file << std::fixed << std::setprecision(kFloatPrecision);
    file << metrics.experiment_id << ','
         << metrics.map_path << ','
         << metrics.scenario_path << ','
         << metrics.num_agents << ','
         << metrics.scenario_index << ','
         << metrics.seed << ','
         << (metrics.solved ? 1 : 0) << ','
         << metrics.makespan_success << ','
         << metrics.total_runtime_ms << ','
         << metrics.total_cnf_clauses << ','
         << metrics.total_cnf_variables << ','
         << metrics.total_mdd_build_time_ms << ','
         << metrics.total_cnf_build_time_ms << ','
         << metrics.total_lazy_wall_time_ms << ','
         << metrics.total_lazy_solver_wall_time_ms << ','
         << metrics.total_lazy_solver_reported_ms
         << '\n';
    finish_file(file, path);
}