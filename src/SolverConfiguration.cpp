#include "lnssat/SolverConfiguration.h"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <regex>

namespace {

std::string trim(const std::string& input) {
    const auto begin = std::find_if_not(
        input.begin(), input.end(), [](unsigned char ch) {
            return std::isspace(ch);
        });
    const auto end = std::find_if_not(
        input.rbegin(), input.rend(), [](unsigned char ch) {
            return std::isspace(ch);
        }).base();
    return begin < end ? std::string(begin, end) : std::string();
}

std::string lower(std::string value) {
    std::transform(
        value.begin(), value.end(), value.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
    return value;
}

std::optional<int> parse_int(const std::string& text) {
    if (text.empty()) {
        return std::nullopt;
    }
    int value = 0;
    const char* begin = text.data();
    const char* end = begin + text.size();
    const auto result = std::from_chars(begin, end, value);
    if (result.ec != std::errc() || result.ptr != end) {
        return std::nullopt;
    }
    return value;
}

std::optional<double> parse_double(const std::string& text) {
    if (text.empty()) {
        return std::nullopt;
    }
    static const std::regex decimal(R"(-?(0|[1-9][0-9]*)(\.[0-9]+)?([eE][+-]?[0-9]+)?)");
    if (!std::regex_match(text, decimal)) return std::nullopt;
    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (errno == ERANGE || end != text.c_str() + text.size() ||
        !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

ConfigurationResolution invalid_resolution(const std::string& message) {
    ConfigurationResolution result;
    result.validation.message = message;
    return result;
}

std::optional<std::string> apply_value(
    const std::string& key,
    const std::string& value,
    SolveRequest& request,
    SolverConfig& config) {
    if (key == "map") {
        request.map_path = value;
    } else if (key == "scenario") {
        request.scenario_path = value;
    } else if (key == "num_agents") {
        const auto parsed = parse_int(value);
        if (!parsed) return "Config field num_agents must be an integer";
        request.num_agents = *parsed;
    } else if (key == "scenario_index") {
        const auto parsed = parse_int(value);
        if (!parsed) return "Config field scenario_index must be an integer";
        request.scenario_index = *parsed;
    } else if (key == "seed") {
        const auto parsed = parse_int(value);
        if (!parsed) return "Config field seed must be an integer";
        config.seed = *parsed;
    } else if (key == "variant") {
        if (value != "lns-sat" && value != "initial-radius-2" && value != "fixed-step-2" && value != "increasing-step") return "Unknown canonical variant: " + value;
        const auto parsed = parse_neighborhood_variant(value);
        if (!parsed) return "Unknown neighborhood variant: " + value;
        config.neighborhood_variant = *parsed;
    } else if (key == "makespan_increment") {
        const auto parsed = parse_int(value);
        if (!parsed) return "Config field makespan_increment must be an integer";
        config.makespan_increment = *parsed;
    } else if (key == "makespan_increase_limit") {
        const auto parsed = parse_int(value);
        if (!parsed) {
            return "Config field makespan_increase_limit must be an integer";
        }
        config.makespan_increase_limit = *parsed;
    } else if (key == "makespan_bound") {
        const auto parsed = parse_int(value);
        if (!parsed) return "Config field makespan_bound must be an integer";
        config.makespan_bound = *parsed;
    } else if (key == "lazy_iteration_limit") {
        const auto parsed = parse_int(value);
        if (!parsed) return "Config field lazy_iteration_limit must be an integer";
        config.lazy_iteration_limit = *parsed;
    } else if (key == "full_map_fallback_threshold") {
        const auto parsed = parse_double(value);
        if (!parsed) {
            return "Config field full_map_fallback_threshold must be numeric";
        }
        config.full_map_fallback_threshold = *parsed;
    } else if (key == "wall_clock_limit_ms") {
        const auto parsed = parse_int(value);
        if (!parsed) return "Config field wall_clock_limit_ms must be an integer";
        config.wall_clock_limit = std::chrono::milliseconds(*parsed);
    } else if (key == "log_level") {
        const auto parsed = parse_log_level(value);
        if (!parsed) return "Unknown log level: " + value;
        config.log_level = *parsed;
    } else if (key == "log") {
        config.log_output_path = value;
    } else {
        return "Unknown configuration key: " + key;
    }
    return std::nullopt;
}

}  // namespace

std::optional<std::string> apply_solver_configuration_value(const std::string& key, const std::string& value, SolveRequest& request, SolverConfig& config) {
    return apply_value(key, value, request, config);
}

std::optional<LogLevel> parse_log_level(const std::string& value) {
    const std::string normalized = lower(trim(value));
    if (normalized == "quiet") return LogLevel::Quiet;
    if (normalized == "info") return LogLevel::Info;
    if (normalized == "debug") return LogLevel::Debug;
    return std::nullopt;
}

const char* log_level_name(LogLevel level) {
    switch (level) {
        case LogLevel::Quiet: return "quiet";
        case LogLevel::Info: return "info";
        case LogLevel::Debug: return "debug";
    }
    return "unknown";
}

ConfigurationValidation validate_solver_configuration(
    const SolveRequest& request,
    const SolverConfig& config) {
    auto invalid = [](const std::string& message) {
        return ConfigurationValidation{false, message};
    };

    if (request.map_path.empty()) {
        return invalid("Map path must not be empty");
    }
    if (request.scenario_path.empty()) {
        return invalid("Scenario path must not be empty");
    }
    if (request.num_agents <= 0) {
        return invalid("Number of agents must be positive");
    }
    if (request.scenario_index < 0) {
        return invalid("Scenario index must be non-negative");
    }
    if (config.makespan_increment <= 0) {
        return invalid("Makespan increment must be positive");
    }
    if (config.makespan_increase_limit < 0) {
        return invalid("Makespan increase limit must be non-negative");
    }
    if (config.makespan_bound && *config.makespan_bound < 0) return invalid("Makespan bound must be nonnegative");
    if (config.lazy_iteration_limit <= 0) {
        return invalid("Lazy iteration limit must be positive");
    }
    if (!std::isfinite(config.full_map_fallback_threshold) ||
        config.full_map_fallback_threshold <= 0.0 ||
        config.full_map_fallback_threshold > 1.0) {
        return invalid("Full-map fallback threshold must be in (0, 1]");
    }
    if (config.wall_clock_limit &&
        config.wall_clock_limit->count() <= 0) {
        return invalid("Wall-clock limit must be positive when specified");
    }

    switch (config.neighborhood_variant) {
        case NeighborhoodVariant::LnsSat:
        case NeighborhoodVariant::InitialRadius2:
        case NeighborhoodVariant::FixedStep2:
        case NeighborhoodVariant::IncreasingStep:
            break;
        default:
            return invalid("Unknown neighborhood variant");
    }
    switch (config.log_level) {
        case LogLevel::Quiet:
        case LogLevel::Info:
        case LogLevel::Debug:
            break;
        default:
            return invalid("Unknown log level");
    }
    return {true, ""};
}

ConfigurationResolution resolve_positional_solve_arguments(
    const std::vector<std::string>& arguments) {
    if (arguments.size() < 4 || arguments.size() > 6) {
        return invalid_resolution(
            "Expected map, scenario, agent count, scenario index, "
            "and optional seed and variant");
    }

    ConfigurationResolution result;
    result.request.map_path = arguments[0];
    result.request.scenario_path = arguments[1];

    const auto agents = parse_int(arguments[2]);
    if (!agents) return invalid_resolution("Agent count must be an integer");
    result.request.num_agents = *agents;

    const auto scenario_index = parse_int(arguments[3]);
    if (!scenario_index) {
        return invalid_resolution("Scenario index must be an integer");
    }
    result.request.scenario_index = *scenario_index;

    if (arguments.size() >= 5) {
        const auto seed = parse_int(arguments[4]);
        if (!seed) return invalid_resolution("Seed must be an integer");
        result.config.seed = *seed;
    }
    if (arguments.size() >= 6) {
        const auto variant = parse_neighborhood_variant(arguments[5]);
        if (!variant) {
            return invalid_resolution(
                "Unknown neighborhood variant: " + arguments[5]);
        }
        result.config.neighborhood_variant = *variant;
    }
    result.validation =
        validate_solver_configuration(result.request, result.config);
    return result;
}

ConfigurationResolution load_solve_configuration_file(
    const std::string& path) {
    try {
        std::ifstream input(path);
        if (!input.is_open()) {
            return invalid_resolution("Failed to open config file: " + path);
        }

        ConfigurationResolution result;
        std::map<std::string, std::string> values;
        std::string line;
        std::size_t line_number = 0;
        while (std::getline(input, line)) {
            ++line_number;
            line = trim(line);
            if (line.empty() || line.front() == '#') {
                continue;
            }
            const auto delimiter = line.find('=');
            if (delimiter == std::string::npos) {
                return invalid_resolution(
                    "Invalid config line " + std::to_string(line_number));
            }
            const std::string key = trim(line.substr(0, delimiter));
            const std::string value = trim(line.substr(delimiter + 1));
            if (key.empty()) {
                return invalid_resolution(
                    "Empty config key on line " + std::to_string(line_number));
            }
            if (!values.emplace(key, value).second) return invalid_resolution("Duplicate configuration key: " + key);
            if (value.empty()) return invalid_resolution("Empty configuration value: " + key);
        }

        if (input.bad()) return invalid_resolution("Failed reading config file");
        for (const auto& entry : values) {
            const auto error = apply_value(
                entry.first, entry.second, result.request, result.config);
            if (error) {
                return invalid_resolution(*error);
            }
        }
        result.validation =
            validate_solver_configuration(result.request, result.config);
        return result;
    } catch (const std::exception& error) {
        return invalid_resolution(
            "Failed to parse config file: " + std::string(error.what()));
    } catch (...) {
        return invalid_resolution("Failed to parse config file");
    }
}

SolverDeadline make_solver_deadline(const SolverConfig& config) {
    if (!config.wall_clock_limit) {
        return std::nullopt;
    }
    return std::chrono::steady_clock::now() + *config.wall_clock_limit;
}
