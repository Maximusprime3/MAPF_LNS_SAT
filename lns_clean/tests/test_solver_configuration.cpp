#include "../SolverConfiguration.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <unistd.h>

namespace {

class TestRunner {
public:
    void expect(bool condition, const std::string& message) {
        ++checks_;
        if (!condition) {
            ++failures_;
            std::cerr << "FAIL: " << message << std::endl;
        }
    }

    int finish() const {
        if (failures_ == 0) {
            std::cout << "PASS: " << checks_
                      << " solver configuration checks" << std::endl;
            return 0;
        }
        std::cerr << "FAIL: " << failures_ << " of " << checks_
                  << " solver configuration checks failed" << std::endl;
        return 1;
    }

private:
    int checks_ = 0;
    int failures_ = 0;
};

class TemporaryConfig {
public:
    explicit TemporaryConfig(const std::string& contents) {
        path_ = std::filesystem::temp_directory_path() /
                ("lns-solver-config-" + std::to_string(::getpid()) + ".ini");
        std::ofstream output(path_);
        output << contents;
    }

    ~TemporaryConfig() {
        std::error_code error;
        std::filesystem::remove(path_, error);
    }

    std::string path() const {
        return path_.string();
    }

private:
    std::filesystem::path path_;
};

ConfigurationResolution load_text(const std::string& contents) {
    TemporaryConfig config(contents);
    return load_solve_configuration_file(config.path());
}

const std::string kRequiredConfig =
    "map=maps/example.map\n"
    "scenario=scenarios/example.scen\n"
    "num_agents=4\n"
    "scenario_index=0\n";

void test_defaults_and_overrides(TestRunner& tests) {
    const auto defaults = resolve_positional_solve_arguments(
        {"maps/example.map", "scenarios/example.scen", "4", "0"});
    tests.expect(defaults.valid(), "valid positional defaults were rejected");
    tests.expect(defaults.config.seed == 42, "default seed changed");
    tests.expect(
        defaults.config.neighborhood_variant == NeighborhoodVariant::LnsSat,
        "default neighborhood variant changed");
    tests.expect(defaults.config.makespan_increment == 1,
                 "default makespan increment changed");
    tests.expect(defaults.config.makespan_increase_limit == 10,
                 "default makespan increase limit changed");
    tests.expect(defaults.config.lazy_iteration_limit == 10000000,
                 "active lazy iteration limit changed");
    tests.expect(defaults.config.full_map_fallback_threshold == 0.95,
                 "default full-map fallback threshold changed");
    tests.expect(!defaults.config.wall_clock_limit.has_value(),
                 "default wall-clock policy should remain unlimited");
    tests.expect(defaults.config.log_level == LogLevel::Info,
                 "default log level changed");

    const auto overrides = load_text(
        kRequiredConfig +
        "seed=73\n"
        "variant=fixed-step-2\n"
        "makespan_increment=2\n"
        "makespan_increase_limit=6\n"
        "lazy_iteration_limit=17\n"
        "full_map_fallback_threshold=0.75\n"
        "wall_clock_limit_ms=250\n"
        "log_level=debug\n"
        "log=solver.log\n");
    tests.expect(overrides.valid(), "explicit valid overrides were rejected");
    tests.expect(overrides.config.seed == 73,
                 "explicit seed override was not resolved");
    tests.expect(
        overrides.config.neighborhood_variant == NeighborhoodVariant::FixedStep2,
        "explicit variant override was not resolved");
    tests.expect(overrides.config.makespan_increment == 2 &&
                     overrides.config.makespan_increase_limit == 6,
                 "explicit makespan policy was not resolved");
    tests.expect(overrides.config.lazy_iteration_limit == 17,
                 "explicit lazy limit was not resolved");
    tests.expect(overrides.config.full_map_fallback_threshold == 0.75,
                 "explicit fallback threshold was not resolved");
    tests.expect(overrides.config.wall_clock_limit ==
                     std::chrono::milliseconds(250),
                 "explicit wall-clock limit was not resolved");
    tests.expect(overrides.config.log_level == LogLevel::Debug &&
                     overrides.config.log_output_path == "solver.log",
                 "explicit logging policy was not resolved");
}

void test_invalid_values(TestRunner& tests) {
    SolveRequest request{"map", "scenario", 4, 0};
    SolverConfig config;

    config.makespan_increment = 0;
    tests.expect(!validate_solver_configuration(request, config).valid,
                 "zero makespan increment was accepted");
    config = SolverConfig{};
    config.makespan_increment = -1;
    tests.expect(!validate_solver_configuration(request, config).valid,
                 "negative makespan increment was accepted");
    config = SolverConfig{};
    config.makespan_increase_limit = -1;
    tests.expect(!validate_solver_configuration(request, config).valid,
                 "negative makespan increase limit was accepted");
    config = SolverConfig{};
    config.lazy_iteration_limit = 0;
    tests.expect(!validate_solver_configuration(request, config).valid,
                 "zero lazy iteration limit was accepted");
    config.lazy_iteration_limit = -1;
    tests.expect(!validate_solver_configuration(request, config).valid,
                 "negative lazy iteration limit was accepted");
    config = SolverConfig{};
    config.wall_clock_limit = std::chrono::milliseconds(0);
    tests.expect(!validate_solver_configuration(request, config).valid,
                 "zero wall-clock limit was accepted");
    config.wall_clock_limit = std::chrono::milliseconds(-1);
    tests.expect(!validate_solver_configuration(request, config).valid,
                 "negative wall-clock limit was accepted");

    for (double threshold :
         {0.0, -0.1, 1.01, std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::quiet_NaN()}) {
        config = SolverConfig{};
        config.full_map_fallback_threshold = threshold;
        tests.expect(!validate_solver_configuration(request, config).valid,
                     "invalid fallback threshold was accepted");
    }

    request.num_agents = 0;
    tests.expect(!validate_solver_configuration(request, SolverConfig{}).valid,
                 "zero agents were accepted");
    request.num_agents = -1;
    tests.expect(!validate_solver_configuration(request, SolverConfig{}).valid,
                 "negative agents were accepted");
    request.num_agents = 4;
    request.scenario_index = -1;
    tests.expect(!validate_solver_configuration(request, SolverConfig{}).valid,
                 "negative scenario index was accepted");

    config = SolverConfig{};
    config.neighborhood_variant =
        static_cast<NeighborhoodVariant>(999);
    tests.expect(!validate_solver_configuration(
                     SolveRequest{"map", "scenario", 4, 0}, config).valid,
                 "unknown typed variant was accepted");
    config = SolverConfig{};
    config.log_level = static_cast<LogLevel>(999);
    tests.expect(!validate_solver_configuration(
                     SolveRequest{"map", "scenario", 4, 0}, config).valid,
                 "unknown typed log level was accepted");
}

void test_parsing_failures(TestRunner& tests) {
    tests.expect(
        !resolve_positional_solve_arguments(
             {"map", "scenario", "four", "0"}).valid(),
        "malformed positional agent count was accepted");
    tests.expect(
        !resolve_positional_solve_arguments(
             {"map", "scenario", "4", "zero"}).valid(),
        "malformed positional scenario index was accepted");
    tests.expect(
        !resolve_positional_solve_arguments(
             {"map", "scenario", "4", "0", "seed"}).valid(),
        "malformed positional seed was accepted");
    tests.expect(
        !resolve_positional_solve_arguments(
             {"map", "scenario", "4", "0", "42", "unknown"}).valid(),
        "unknown positional variant was accepted");
    tests.expect(
        !resolve_positional_solve_arguments(
             {"map", "scenario", "4", "0", "minisat", "42"}).valid(),
        "obsolete minisat positional slot was accepted");
    tests.expect(
        !resolve_positional_solve_arguments(
             {"map", "scenario", "4", "0", "probsat", "42"}).valid(),
        "obsolete probsat positional slot was accepted");
    tests.expect(!load_text(kRequiredConfig + "solver=minisat\n").valid(),
                 "obsolete config solver key was accepted");
    tests.expect(!load_text(kRequiredConfig + "variant=unknown\n").valid(),
                 "unknown config variant was accepted");
    tests.expect(!load_text(kRequiredConfig + "log_level=trace\n").valid(),
                 "unknown config log level was accepted");

    const std::vector<std::pair<std::string, std::string>> malformed{
        {"num_agents", "4x"},
        {"scenario_index", "0x"},
        {"seed", "999999999999999999999"},
        {"makespan_increment", "1.5"},
        {"makespan_increase_limit", "--1"},
        {"lazy_iteration_limit", "many"},
        {"full_map_fallback_threshold", "0.9x"},
        {"wall_clock_limit_ms", "soon"},
    };
    for (const auto& [key, value] : malformed) {
        tests.expect(
            !load_text(kRequiredConfig + key + "=" + value + "\n").valid(),
            "malformed numeric config value was accepted for " + key);
    }
}

void test_equivalent_inputs(TestRunner& tests) {
    const auto positional = resolve_positional_solve_arguments(
        {"maps/example.map", "scenarios/example.scen", "4", "0",
         "73", "fixed-step-2"});
    const auto file = load_text(
        kRequiredConfig +
        "seed=73\n"
        "variant=fixed-step-2\n");
    tests.expect(positional.valid() && file.valid(),
                 "equivalent input forms should both resolve");
    tests.expect(positional.request == file.request,
                 "positional and file requests differ");
    tests.expect(positional.config == file.config,
                 "positional and file solver configs differ");
}

}  // namespace

int main() {
    TestRunner tests;
    test_defaults_and_overrides(tests);
    test_invalid_values(tests);
    test_parsing_failures(tests);
    test_equivalent_inputs(tests);
    return tests.finish();
}
