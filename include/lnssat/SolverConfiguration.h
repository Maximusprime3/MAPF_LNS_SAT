#pragma once

#include "lnssat/NeighborhoodVariant.h"
#include "lnssat/Deadline.h"

#include <chrono>
#include <optional>
#include <string>
#include <vector>

enum class LogLevel {
    Quiet,
    Info,
    Debug,
};

struct SolveRequest {
    std::string map_path;
    std::string scenario_path;
    int num_agents = -1;
    int scenario_index = 0;

    bool operator==(const SolveRequest& other) const {
        return map_path == other.map_path &&
               scenario_path == other.scenario_path &&
               num_agents == other.num_agents &&
               scenario_index == other.scenario_index;
    }
};

struct SolverConfig {
    int seed = 42;
    NeighborhoodVariant neighborhood_variant = NeighborhoodVariant::LnsSat;
    int makespan_increment = 1;
    int makespan_increase_limit = 10;
    int lazy_iteration_limit = 10000000;
    double full_map_fallback_threshold = 0.95;
    std::optional<std::chrono::milliseconds> wall_clock_limit;
    LogLevel log_level = LogLevel::Info;
    std::string log_output_path;

    bool operator==(const SolverConfig& other) const {
        return seed == other.seed &&
               neighborhood_variant == other.neighborhood_variant &&
               makespan_increment == other.makespan_increment &&
               makespan_increase_limit == other.makespan_increase_limit &&
               lazy_iteration_limit == other.lazy_iteration_limit &&
               full_map_fallback_threshold == other.full_map_fallback_threshold &&
               wall_clock_limit == other.wall_clock_limit &&
               log_level == other.log_level &&
               log_output_path == other.log_output_path;
    }
};

struct ConfigurationValidation {
    bool valid = false;
    std::string message;
};

struct ConfigurationResolution {
    SolveRequest request;
    SolverConfig config;
    ConfigurationValidation validation;

    bool valid() const {
        return validation.valid;
    }
};



ConfigurationValidation validate_solver_configuration(
    const SolveRequest& request,
    const SolverConfig& config);

ConfigurationResolution resolve_positional_solve_arguments(
    const std::vector<std::string>& arguments);

ConfigurationResolution load_solve_configuration_file(
    const std::string& path);

std::optional<LogLevel> parse_log_level(const std::string& value);
const char* log_level_name(LogLevel level);

SolverDeadline make_solver_deadline(const SolverConfig& config);
