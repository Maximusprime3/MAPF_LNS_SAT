#pragma once

#include <string>
#include <vector>

inline double seconds_to_milliseconds(double seconds) {
    return seconds * 1000.0;
}

struct LazySatIterationMetrics {
    int iteration = 0;
    int clause_count_before = 0;
    int variable_count = 0;
    int clauses_added = 0;
    int total_clauses_after = 0;
    int new_vertex_collisions = 0;
    int new_edge_collisions = 0;
    int total_vertex_collisions = 0;
    int total_edge_collisions = 0;
    double iteration_wall_time_ms = 0.0;
    double solver_wall_time_ms = 0.0;
    double solver_reported_time_ms = 0.0;
    bool used_assumptions = false;
    bool reset_solver = false;
    bool satisfiable = false;
    int num_decisions = 0;
    int num_propagations = 0;
};

struct LazySolveRunMetrics {
    std::vector<LazySatIterationMetrics> iterations;
    double total_wall_time_ms = 0.0;
    double total_solver_wall_time_ms = 0.0;
    double total_solver_reported_time_ms = 0.0;
    int final_clause_count = 0;
    int final_variable_count = 0;
    bool solved = false;
};

struct WaitingAttemptMetrics {
    int attempt_index = 0;
    int waiting_time_budget = 0;
    int zone_positions = 0;
    int segment_count = 0;
    int agent_count = 0;
    int start_t = 0;
    int end_t = 0;
    int cnf_clauses = 0;
    int cnf_variables = 0;
    double mdd_build_time_ms = 0.0;
    double cnf_build_time_ms = 0.0;
    double attempt_wall_time_ms = 0.0;
    bool applied_waiting_time = false;
    bool extended_time_window = false;
    LazySolveRunMetrics lazy_metrics;
    bool solved = false;
};

struct LocalZoneAttemptMetrics {
    int attempt_index = 0;
    size_t zone_positions = 0;
    double zone_fraction = 0.0;
    int conflicts = 0;
    int agents_in_window = 0;
    int start_t = 0;
    int end_t = 0;
    int waiting_attempts = 0;
    int total_lazy_iterations = 0;
    long long total_cnf_clauses = 0;
    long long total_cnf_variables = 0;
    double total_mdd_build_time_ms = 0.0;
    double total_cnf_build_time_ms = 0.0;
    double total_lazy_wall_time_ms = 0.0;
    double total_lazy_solver_wall_time_ms = 0.0;
    double total_lazy_solver_reported_time_ms = 0.0;
    bool solved = false;
};

struct MakespanAttemptMetrics {
    int attempt_index = 0;
    int makespan = 0;
    int zones_attempted = 0;
    int zones_solved = 0;
    int total_waiting_attempts = 0;
    int total_lazy_iterations = 0;
    long long total_cnf_clauses = 0;
    long long total_cnf_variables = 0;
    double total_mdd_build_time_ms = 0.0;
    double total_cnf_build_time_ms = 0.0;
    double total_lazy_wall_time_ms = 0.0;
    double total_lazy_solver_wall_time_ms = 0.0;
    double total_lazy_solver_reported_time_ms = 0.0;
    double attempt_wall_time_ms = 0.0;
    bool solved = false;
};

struct ExperimentSummaryMetrics {
    std::string experiment_id;
    std::string map_path;
    std::string scenario_path;
    int num_agents = 0;
    int scenario_index = 0;
    int seed = 0;
    int makespan_success = -1;
    double total_runtime_ms = 0.0;
    bool solved = false;
    long long total_cnf_clauses = 0;
    long long total_cnf_variables = 0;
    double total_mdd_build_time_ms = 0.0;
    double total_cnf_build_time_ms = 0.0;
    double total_lazy_wall_time_ms = 0.0;
    double total_lazy_solver_wall_time_ms = 0.0;
    double total_lazy_solver_reported_ms = 0.0;
};