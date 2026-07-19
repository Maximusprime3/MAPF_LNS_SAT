#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <thread>
#include <csignal>
#include <unistd.h>
#ifdef __linux__
#include <sys/wait.h>
#endif

#include "simple_json.h"

namespace fs = std::filesystem;

struct Options {
    std::optional<std::string> config_path;
    std::optional<std::string> map_path;
    std::string scenario_dir = "mapf-scen-even/scen-even";
    std::string scenario_pattern = "{map}-even-*.scen";
    std::optional<int> num_agents;
    int experiments = 1;
    std::string solver = "minisat";
    int seed = 42;
    // Reproduce the published LNS-SAT baseline unless a paper variant is requested.
    std::string variant = "lns-sat";
    int start_run_id = 0;
    bool dry_run = false;
    bool verbose = false;
    std::optional<std::string> log_file;
    std::optional<int> time_limit_seconds;
};

struct ScenarioStats {
    fs::path path;
    int entries = 0;
    int experiments_possible = 0;
};

struct ConfiguredRun {
    Options options;
    std::optional<std::string> label;
};

namespace {

[[noreturn]] void usage(const char* program, const std::string& message) {
    if (!message.empty()) {
        std::cerr << "Error: " << message << "\n";
    }
    std::cerr << "Usage: " << program << " [options]\n";
    std::cerr << "Options:\n"
              << "  --config PATH            JSON configuration file with run definitions\n"
              << "  --map PATH               Path to the .map file\n"
              << "  --scenario-dir DIR       Directory containing scenario files (default: mapf-scen-even/scen-even)\n"
              << "  --scenario-pattern PAT   Glob pattern within the scenario directory (default: {map}-even-*.scen)\n"
              << "  --num-agents N           Number of agents per experiment\n"
              << "  --experiments N          Number of experiments to execute (default: 1)\n"
              << "  --solver NAME            minisat or probsat (default: minisat)\n"
              << "  --seed N                 Random seed forwarded to the solver (default: 42)\n"
              << "  --variant NAME           Neighborhood policy: lns-sat, initial-radius-2,\n"
              << "                           fixed-step-2, or increasing-step (default: lns-sat)\n"
              << "  --start-run-id N         Starting run identifier (default: 0)\n"
              << "  --time-limit SECONDS     Maximum wall-clock time per run (0 disables)\n"
              << "  --dry-run                Only report capacities without launching experiments\n"
              << "  --verbose                Print command lines before execution\n"
              << "  --log-file PATH          Append solver output to the given log file\n";
    std::exit(EXIT_FAILURE);
}

bool starts_with(const std::string& text, const std::string& prefix) {
    return text.compare(0, prefix.size(), prefix) == 0;
}

bool equals_ignore_case(const std::string& lhs, const std::string& rhs) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(lhs[i])) !=
            std::tolower(static_cast<unsigned char>(rhs[i]))) {
            return false;
        }
    }
    return true;
}

std::optional<int> parse_int(const std::string& value) {
    try {
        size_t idx = 0;
        long parsed = std::stol(value, &idx, 10);
        if (idx != value.size()) {
            return std::nullopt;
        }
        if (parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
            return std::nullopt;
        }
        return static_cast<int>(parsed);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

Options parse_arguments(int argc, char** argv) {
    Options opts;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto require_value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                usage(argv[0], std::string("Missing value for ") + name);
            }
            return argv[++i];
        };

        if (arg == "--config") {
            opts.config_path = require_value("--config");
        } else if (arg == "--map") {
            opts.map_path = require_value("--map");
        } else if (arg == "--scenario-dir") {
            opts.scenario_dir = require_value("--scenario-dir");
        } else if (arg == "--scenario-pattern") {
            opts.scenario_pattern = require_value("--scenario-pattern");
        } else if (arg == "--num-agents") {
            std::string value = require_value("--num-agents");
            auto parsed = parse_int(value);
            if (!parsed) {
                usage(argv[0], "--num-agents requires an integer");
            }
            opts.num_agents = *parsed;
        } else if (arg == "--experiments") {
            std::string value = require_value("--experiments");
            auto parsed = parse_int(value);
            if (!parsed) {
                usage(argv[0], "--experiments requires an integer");
            }
            opts.experiments = *parsed;
        } else if (arg == "--solver") {
            opts.solver = require_value("--solver");
        } else if (arg == "--variant") {
            opts.variant = require_value("--variant");
        } else if (arg == "--seed") {
            std::string value = require_value("--seed");
            auto parsed = parse_int(value);
            if (!parsed) {
                usage(argv[0], "--seed requires an integer");
            }
            opts.seed = *parsed;
        } else if (arg == "--start-run-id") {
            std::string value = require_value("--start-run-id");
            auto parsed = parse_int(value);
            if (!parsed) {
                usage(argv[0], "--start-run-id requires an integer");
            }
            opts.start_run_id = *parsed;
        } else if (arg == "--time-limit") {
            std::string value = require_value("--time-limit");
            auto parsed = parse_int(value);
            if (!parsed || *parsed < 0) {
                usage(argv[0], "--time-limit requires a non-negative integer");
            }
            if (*parsed == 0) {
                opts.time_limit_seconds.reset();
            } else {
                opts.time_limit_seconds = *parsed;
            }
        } else if (arg == "--dry-run") {
            opts.dry_run = true;
        } else if (arg == "--verbose") {
            opts.verbose = true;
        } else if (arg == "--log-file") {
            opts.log_file = require_value("--log-file");
        } else {
            usage(argv[0], "Unknown argument: " + arg);
        }
    }
    return opts;
}

std::string glob_to_regex(const std::string& pattern) {
    std::string regex = "^";
    for (char c : pattern) {
        switch (c) {
            case '*':
                regex += ".*";
                break;
            case '?':
                regex += '.';
                break;
            case '.':
            case '+':
            case '(': case ')':
            case '{': case '}':
            case '[': case ']':
            case '^': case '$':
            case '|': case '\\':
                regex += '\\';
                regex.push_back(c);
                break;
            default:
                regex.push_back(c);
                break;
        }
    }
    regex += "$";
    return regex;
}

struct Token {
    bool is_number;
    std::string text;
    int number;
};

std::vector<Token> tokenize_filename(const std::string& name) {
    std::vector<Token> tokens;
    std::size_t i = 0;
    while (i < name.size()) {
        if (std::isdigit(static_cast<unsigned char>(name[i]))) {
            std::size_t start = i;
            while (i < name.size() && std::isdigit(static_cast<unsigned char>(name[i]))) {
                ++i;
            }
            int value = std::stoi(name.substr(start, i - start));
            tokens.push_back(Token{true, std::string(), value});
        } else {
            std::size_t start = i;
            while (i < name.size() && !std::isdigit(static_cast<unsigned char>(name[i]))) {
                ++i;
            }
            tokens.push_back(Token{false, name.substr(start, i - start), 0});
        }
    }
    return tokens;
}

bool natural_less(const fs::path& lhs, const fs::path& rhs) {
    auto left = tokenize_filename(lhs.filename().string());
    auto right = tokenize_filename(rhs.filename().string());
    std::size_t max_count = std::max(left.size(), right.size());
    for (std::size_t i = 0; i < max_count; ++i) {
        if (i >= left.size()) {
            return true;
        }
        if (i >= right.size()) {
            return false;
        }
        const Token& ltok = left[i];
        const Token& rtok = right[i];
        if (ltok.is_number && rtok.is_number) {
            if (ltok.number != rtok.number) {
                return ltok.number < rtok.number;
            }
        } else if (ltok.is_number != rtok.is_number) {
            return ltok.is_number;
        } else {
            if (ltok.text != rtok.text) {
                return ltok.text < rtok.text;
            }
        }
    }
    return left.size() < right.size();
}

std::vector<fs::path> resolve_scenarios(const fs::path& directory, const std::string& pattern) {
    std::vector<fs::path> results;
    if (!fs::exists(directory) || !fs::is_directory(directory)) {
        return results;
    }
    std::regex matcher(glob_to_regex(pattern));
    for (auto const& entry : fs::directory_iterator(directory)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const std::string name = entry.path().filename().string();
        if (std::regex_match(name, matcher)) {
            results.push_back(entry.path());
        }
    }
    std::sort(results.begin(), results.end(), natural_less);
    return results;
}

int count_scenario_entries(const fs::path& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open scenario file: " + path.string());
    }
    std::string line;
    int count = 0;
    bool first_line = true;
    while (std::getline(file, line)) {
        if (first_line && !line.empty()) {
            std::string lower = line;
            std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
                return static_cast<char>(std::tolower(c));
            });
            if (starts_with(lower, "version")) {
                first_line = false;
                continue;
            }
        }
        first_line = false;
        if (!line.empty()) {
            ++count;
        }
    }
    return count;
}

std::vector<ScenarioStats> gather_stats(const std::vector<fs::path>& scenarios, int num_agents) {
    std::vector<ScenarioStats> stats;
    stats.reserve(scenarios.size());
    for (const auto& path : scenarios) {
        int entries = count_scenario_entries(path);
        int possible = num_agents > 0 ? entries / num_agents : 0;
        stats.push_back(ScenarioStats{path, entries, possible});
    }
    return stats;
}

void print_stats(const std::vector<ScenarioStats>& stats, int num_agents, int requested) {
    std::cout << "Scenario capacity overview:\n";
    std::cout << "============================\n";
    int total_entries = 0;
    int total_experiments = 0;
    for (const auto& item : stats) {
        total_entries += item.entries;
        total_experiments += item.experiments_possible;
        int leftover = item.entries - item.experiments_possible * num_agents;
        std::cout << "- " << item.path.filename().string() << ": entries=" << item.entries
                  << ", supports=" << item.experiments_possible << " experiments of "
                  << num_agents;
        if (leftover > 0) {
            std::cout << ", leftover=" << leftover;
        }
        std::cout << "\n";
    }
    std::cout << "----------------------------\n";
    std::cout << "Total entries: " << total_entries << " -> supports " << total_experiments
              << " experiments of " << num_agents << " agents\n";
    if (requested >= 0) {
        std::cout << "Requested experiments: " << requested << "\n";
        if (requested > total_experiments) {
            std::cout << "WARNING: Requested experiments exceed available combinations; some runs will not be executed.\n";
        }
    }
    std::cout << "\n";
}

fs::path check_executable(const std::string& solver) {
    if (!equals_ignore_case(solver, "minisat") && !equals_ignore_case(solver, "probsat")) {
        throw std::runtime_error("Unknown solver: " + solver);
    }

    const std::vector<fs::path> candidates = {
        fs::path("lns_clean") / "main_clean_lns",
        fs::path("main_clean_lns"),
        fs::path("..") / "main_clean_lns",
    };

    for (const auto& candidate : candidates) {
        if (!fs::exists(candidate) || !fs::is_regular_file(candidate)) {
            continue;
        }
        fs::path absolute = fs::absolute(candidate);
        if (::access(absolute.c_str(), X_OK) == 0) {
            return absolute;
        }
    }

    std::ostringstream oss;
    oss << "Required solver executable 'main_clean_lns' not found or not executable. "
        << "Build it with: make -C lns_clean";
    throw std::runtime_error(oss.str());
}

[[maybe_unused]] std::string shell_quote(const std::string& arg) {
    std::string result = "'";
    for (char c : arg) {
        if (c == '\'') {
            result += "'\\''";
        } else {
            result.push_back(c);
        }
    }
    result += "'";
    return result;
}

int run_command(const std::vector<std::string>& command, std::ostream* log_stream,
                std::optional<std::chrono::seconds> time_limit) {
#ifdef __linux__
    std::vector<char*> args;
    args.reserve(command.size() + 1);
    for (const auto& part : command) {
        args.push_back(const_cast<char*>(part.c_str()));
    }
    args.push_back(nullptr);

    int pipefd[2];
    if (::pipe(pipefd) != 0) {
        return -1;
    }

    pid_t pid = ::fork();
    if (pid == -1) {
        ::close(pipefd[0]);
        ::close(pipefd[1]);
        return -1;
    }

    if (pid == 0) {
        ::close(pipefd[0]);
        ::dup2(pipefd[1], STDOUT_FILENO);
        ::dup2(pipefd[1], STDERR_FILENO);
        ::close(pipefd[1]);
        ::execvp(args[0], args.data());
        std::perror("execvp");
        std::_Exit(127);
    }

    ::close(pipefd[1]);
    FILE* stream = ::fdopen(pipefd[0], "r");
    if (!stream) {
        ::close(pipefd[0]);
        ::kill(pid, SIGKILL);
        int status_dummy = 0;
        ::waitpid(pid, &status_dummy, 0);
        return -1;
    }

    std::thread reader([stream, log_stream]() {
        char buffer[4096];
        while (std::fgets(buffer, sizeof(buffer), stream)) {
            if (log_stream) {
                (*log_stream) << buffer;
                log_stream->flush();
            } else {
                std::cout << buffer;
                std::cout.flush();
            }
        }
        std::fclose(stream);
    });

    const bool has_limit = time_limit && time_limit->count() > 0;
    const auto deadline = has_limit ? std::chrono::steady_clock::now() + *time_limit
                                    : std::chrono::steady_clock::time_point::max();
    int status = 0;
    bool timed_out = false;
    while (true) {
        pid_t result = ::waitpid(pid, &status, WNOHANG);
        if (result == pid) {
            break;
        }
        if (result == -1) {
            status = -1;
            break;
        }
        if (has_limit && std::chrono::steady_clock::now() >= deadline) {
            timed_out = true;
            ::kill(pid, SIGKILL);
            ::waitpid(pid, &status, 0);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (reader.joinable()) {
        reader.join();
    }

    if (timed_out) {
        if (log_stream) {
            (*log_stream) << "Command terminated after exceeding the time limit.\n";
            log_stream->flush();
        } else {
            std::cout << "Command terminated after exceeding the time limit.\n";
        }
        return 124;
    }
    if (status == -1) {
        return -1;
    }
    if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
    }
    if (WIFSIGNALED(status)) {
        return 128 + WTERMSIG(status);
    }
    return status;
#else
    (void)time_limit;
    std::string command_line;
    for (std::size_t i = 0; i < command.size(); ++i) {
        if (i > 0) {
            command_line.push_back(' ');
        }
        command_line += shell_quote(command[i]);
    }
    command_line += " 2>&1";
    FILE* pipe = ::popen(command_line.c_str(), "r");
    if (!pipe) {
        return -1;
    }
    char buffer[4096];
    while (std::fgets(buffer, sizeof(buffer), pipe)) {
        if (log_stream) {
            (*log_stream) << buffer;
            log_stream->flush();
        } else {
            std::cout << buffer;
        }
    }
    int status = ::pclose(pipe);
    if (status == -1) {
        return -1;
    }
#ifdef WIFEXITED
    if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
    }
#endif
    return status;
#endif
}

void append_log_header(std::ofstream& log, const Options& opts, const fs::path& map) {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_ptr = std::localtime(&now_time);
    if (tm_ptr) {
        log << "# Batch started " << std::put_time(tm_ptr, "%Y-%m-%d %H:%M:%S");
    } else {
        log << "# Batch started (timestamp=" << now_time << ")";
    }
    log << " | map=" << map.string() << " | agents=" << *opts.num_agents;
    if (opts.time_limit_seconds && *opts.time_limit_seconds > 0) {
        log << " | total_time_limit=" << *opts.time_limit_seconds << "s";
    }
    // Persist the policy beside every batch so results remain reproducible.
    log << " | variant=" << opts.variant << "\n";
    log.flush();
}

std::vector<std::string> build_command(const fs::path& exe, const Options& opts,
                                       const fs::path& scenario_path, int scenario_index) {
    std::vector<std::string> cmd;
    cmd.push_back(exe.string());
    cmd.push_back(*opts.map_path);
    cmd.push_back(scenario_path.string());
    cmd.push_back(std::to_string(*opts.num_agents));
    cmd.push_back(std::to_string(scenario_index));
    cmd.push_back(opts.solver);
    cmd.push_back(std::to_string(opts.seed));
    // main_clean_lns accepts the neighborhood variant after the optional seed.
    cmd.push_back(opts.variant);
    return cmd;
}

int execute_single_run(Options opts, const std::optional<std::string>& label) {
    if (!opts.num_agents || *opts.num_agents <= 0) {
        std::cerr << "Error: number of agents must be a positive integer (CLI or config).\n";
        return EXIT_FAILURE;
    }
    if (!opts.map_path) {
        std::cerr << "Error: map path must be specified (CLI or config).\n";
        return EXIT_FAILURE;
    }
    if (opts.experiments < 0) {
        std::cerr << "Error: --experiments cannot be negative.\n";
        return EXIT_FAILURE;
    }
    fs::path map_path(*opts.map_path);
    if (!fs::exists(map_path)) {
        std::cerr << "Error: map file '" << map_path << "' does not exist.\n";
        return EXIT_FAILURE;
    }
    fs::path scenario_dir(opts.scenario_dir);
    if (!fs::exists(scenario_dir) || !fs::is_directory(scenario_dir)) {
        std::cerr << "Error: scenario directory '" << scenario_dir << "' does not exist.\n";
        return EXIT_FAILURE;
    }
    std::string map_base = map_path.stem().string();
    std::string pattern = opts.scenario_pattern;
    std::size_t pos = pattern.find("{map}");
    if (pos != std::string::npos) {
        pattern.replace(pos, 5, map_base);
    }
    auto scenarios = resolve_scenarios(scenario_dir, pattern);
    if (scenarios.empty()) {
        std::cerr << "Error: no scenario files matching pattern '" << pattern << "' in "
                  << scenario_dir << "\n";
        return EXIT_FAILURE;
    }
    auto stats = gather_stats(scenarios, *opts.num_agents);
    if (label) {
        std::cout << "Configuration '" << *label << "': map=" << map_path << ", agents="
                  << *opts.num_agents << ", experiments=" << opts.experiments << "\n";
    }
    std::optional<std::chrono::seconds> total_time_limit;
    std::optional<std::chrono::steady_clock::time_point> run_start;
    std::optional<std::chrono::steady_clock::time_point> run_deadline;
    if (opts.time_limit_seconds && *opts.time_limit_seconds > 0) {
        total_time_limit = std::chrono::seconds(*opts.time_limit_seconds);
        std::cout << "Overall run time limit: " << total_time_limit->count() << " seconds\n";
    }
    print_stats(stats, *opts.num_agents, opts.experiments);

    if (opts.dry_run || opts.experiments == 0) {
        std::cout << "Dry run requested; no experiments executed.\n";
        return EXIT_SUCCESS;
    }

    fs::path exe_path;
    try {
        exe_path = check_executable(opts.solver);
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return EXIT_FAILURE;
    }

    std::ofstream log_stream;
    if (opts.log_file) {
        fs::path log_path(*opts.log_file);
        if (!log_path.is_absolute()) {
            log_path = fs::absolute(log_path);
        }
        if (log_path.has_parent_path()) {
            fs::create_directories(log_path.parent_path());
        }
        log_stream.open(log_path, std::ios::app);
        if (!log_stream.is_open()) {
            std::cerr << "Error: unable to open log file '" << log_path << "' for writing.\n";
            return EXIT_FAILURE;
        }
        append_log_header(log_stream, opts, map_path);
        std::cout << "Appending solver output to " << log_path << "\n";
    }

    if (total_time_limit) {
        run_start = std::chrono::steady_clock::now();
        run_deadline = *run_start + *total_time_limit;
    }

    int remaining = opts.experiments;
    int run_id = opts.start_run_id;
    bool aborted_due_to_time_limit = false;
    for (const auto& item : stats) {
        if (remaining <= 0 || aborted_due_to_time_limit) {
            break;
        }
        if (item.experiments_possible <= 0) {
            continue;
        }
        int runs_here = std::min(item.experiments_possible, remaining);
        std::cout << "Executing " << runs_here << " experiment(s) on "
                  << item.path.filename().string() << " (" << *opts.num_agents
                  << " agents per chunk).\n";
        for (int local = 0; local < runs_here; ++local) {
            if (aborted_due_to_time_limit) {
                break;
            }
            std::optional<std::chrono::seconds> remaining_time;
            if (run_deadline) {
                auto now = std::chrono::steady_clock::now();
                if (now >= *run_deadline) {
                    aborted_due_to_time_limit = true;
                    break;
                }
                auto diff = std::chrono::duration_cast<std::chrono::seconds>(*run_deadline - now);
                if (diff.count() <= 0) {
                    aborted_due_to_time_limit = true;
                    break;
                }
                remaining_time = diff;
            }
            int scenario_index = local;
            auto cmd = build_command(exe_path, opts, item.path, scenario_index);
            if (opts.verbose) {
                std::ostringstream oss;
                for (std::size_t i = 0; i < cmd.size(); ++i) {
                    if (i > 0) {
                        oss << ' ';
                    }
                    oss << cmd[i];
                }
                std::cout << "  Command: " << oss.str() << "\n";
            } else {
                std::cout << "  Run #" << run_id << ": scenario_index=" << scenario_index << " -> "
                          << item.path.filename().string();
                if (remaining_time) {
                    std::cout << " (remaining_time=" << remaining_time->count() << "s)";
                }
                std::cout << "\n";
            }
            if (log_stream.is_open()) {
                log_stream << "\n# Run " << run_id << " | scenario=" << item.path
                           << " | scenario_index=" << scenario_index << "\n";
                std::ostringstream oss;
                for (std::size_t i = 0; i < cmd.size(); ++i) {
                    if (i > 0) {
                        oss << ' ';
                    }
                    oss << cmd[i];
                }
                log_stream << "# Command: " << oss.str() << "\n";
                if (remaining_time) {
                    log_stream << "# Remaining time budget: " << remaining_time->count()
                               << " seconds\n";
                }
                log_stream.flush();
            }
            int exit_code = run_command(cmd, log_stream.is_open() ? &log_stream : nullptr,
                                        remaining_time);
            if (run_deadline && exit_code == 124) {
                if (log_stream.is_open()) {
                    log_stream << "# Run terminated after reaching overall time limit\n";
                    log_stream.flush();
                }
                std::cout << "  Overall time limit reached; stopping further experiments.\n";
                aborted_due_to_time_limit = true;
                break;
            }
            if (exit_code != 0) {
                std::cerr << "Error: experiment run_id=" << run_id
                          << " failed with exit code " << exit_code << "\n";
                if (log_stream.is_open()) {
                    log_stream << "# Run failed with exit code " << exit_code << "\n";
                    log_stream.flush();
                    log_stream << "# Batch aborted due to failure\n";
                    log_stream.flush();
                }
                return EXIT_FAILURE;
            }
            ++run_id;
            --remaining;
            if (remaining <= 0) {
                break;
            }
        }
    }

    if (aborted_due_to_time_limit) {
        int completed = opts.experiments - remaining;
        std::optional<std::chrono::seconds> elapsed_seconds;
        if (run_start) {
            elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - *run_start);
        }
        std::cout << "Stopped after reaching the overall time limit with " << completed
                  << " completed experiment(s)";
        if (elapsed_seconds) {
            std::cout << " (elapsed " << elapsed_seconds->count() << "s)";
        }
        std::cout << ".\n";
        if (log_stream.is_open()) {
            log_stream << "\n# Batch stopped after reaching overall time limit";
            if (elapsed_seconds) {
                log_stream << " (elapsed " << elapsed_seconds->count() << "s)";
            }
            log_stream << "\n";
            log_stream.flush();
        }
        return EXIT_SUCCESS;
    }

    if (remaining > 0) {
        std::cout << "Stopped early: " << remaining
                  << " experiment(s) could not be scheduled due to limited scenario entries.\n";
        if (log_stream.is_open()) {
            log_stream << "\n# Batch finished with incomplete status\n";
            log_stream.flush();
        }
        return EXIT_SUCCESS;
    }

    std::cout << "All requested experiments completed.\n";
    if (log_stream.is_open()) {
        log_stream << "\n# Batch finished successfully\n";
        log_stream.flush();
    }
    return EXIT_SUCCESS;
}

bool truthy(const simple_json::JsonValue& value) {
    if (value.is_bool()) {
        return value.as_bool();
    }
    if (value.is_number()) {
        return value.as_number() != 0.0;
    }
    if (value.is_string()) {
        std::string text = value.as_string();
        std::string lowered;
        lowered.resize(text.size());
        std::transform(text.begin(), text.end(), lowered.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return lowered == "1" || lowered == "true" || lowered == "yes" || lowered == "on";
    }
    return false;
}

std::optional<int> value_to_int(const simple_json::JsonValue& value) {
    if (value.is_number()) {
        double number = value.as_number();
        double integral;
        if (std::modf(number, &integral) == 0.0) {
            if (integral >= std::numeric_limits<int>::min() &&
                integral <= std::numeric_limits<int>::max()) {
                return static_cast<int>(integral);
            }
        }
    }
    if (value.is_string()) {
        return parse_int(value.as_string());
    }
    return std::nullopt;
}

std::optional<std::string> value_to_string(const simple_json::JsonValue& value) {
    if (value.is_string()) {
        return value.as_string();
    }
    return std::nullopt;
}

fs::path resolve_config_path(const fs::path& base_dir, const std::string& value) {
    fs::path input(value);
    if (input.is_absolute()) {
        return input;
    }
    fs::path combined = base_dir / input;
    try {
        return fs::weakly_canonical(combined);
    } catch (const fs::filesystem_error&) {
        return fs::absolute(combined);
    }
}

Options apply_config_entry(const Options& base, const simple_json::JsonObject& entry,
                           const fs::path& base_dir, std::optional<std::string>& label_out) {
    Options result = base;
    label_out.reset();
    static const std::map<std::string, int> allowed = {
        {"map", 0},
        {"scenario_dir", 0},
        {"scenario_pattern", 0},
        {"num_agents", 0},
        {"experiments", 0},
        {"solver", 0},
        {"seed", 0},
        {"variant", 0},
        {"start_run_id", 0},
        {"dry_run", 0},
        {"verbose", 0},
        {"log_file", 0},
        {"name", 0},
        {"time_limit_seconds", 0},
    };
    for (const auto& kv : entry) {
        if (!allowed.count(kv.first)) {
            throw std::runtime_error("Unknown key in configuration: " + kv.first);
        }
    }
    for (const auto& kv : entry) {
        const std::string& key = kv.first;
        const simple_json::JsonValue& value = kv.second;
        if (key == "name") {
            if (!value.is_string()) {
                throw std::runtime_error("Configuration field 'name' must be a string");
            }
            label_out = value.as_string();
        } else if (key == "map") {
            auto str = value_to_string(value);
            if (!str) {
                throw std::runtime_error("Configuration field 'map' must be a string");
            }
            fs::path resolved = resolve_config_path(base_dir, *str);
            result.map_path = resolved.string();
        } else if (key == "scenario_dir") {
            auto str = value_to_string(value);
            if (!str) {
                throw std::runtime_error("Configuration field 'scenario_dir' must be a string");
            }
            fs::path resolved = resolve_config_path(base_dir, *str);
            result.scenario_dir = resolved.string();
        } else if (key == "scenario_pattern") {
            auto str = value_to_string(value);
            if (!str) {
                throw std::runtime_error("Configuration field 'scenario_pattern' must be a string");
            }
            result.scenario_pattern = *str;
        } else if (key == "num_agents") {
            auto parsed = value_to_int(value);
            if (!parsed) {
                throw std::runtime_error("Configuration field 'num_agents' must be an integer");
            }
            result.num_agents = *parsed;
        } else if (key == "experiments") {
            auto parsed = value_to_int(value);
            if (!parsed) {
                throw std::runtime_error("Configuration field 'experiments' must be an integer");
            }
            result.experiments = *parsed;
        } else if (key == "solver") {
            auto str = value_to_string(value);
            if (!str) {
                throw std::runtime_error("Configuration field 'solver' must be a string");
            }
            result.solver = *str;
        } else if (key == "variant") {
            auto str = value_to_string(value);
            if (!str) {
                throw std::runtime_error("Configuration field 'variant' must be a string");
            }
            result.variant = *str;
        } else if (key == "seed") {
            auto parsed = value_to_int(value);
            if (!parsed) {
                throw std::runtime_error("Configuration field 'seed' must be an integer");
            }
            result.seed = *parsed;
        } else if (key == "start_run_id") {
            auto parsed = value_to_int(value);
            if (!parsed) {
                throw std::runtime_error("Configuration field 'start_run_id' must be an integer");
            }
            result.start_run_id = *parsed;
        } else if (key == "time_limit_seconds") {
            auto parsed = value_to_int(value);
            if (!parsed || *parsed < 0) {
                throw std::runtime_error(
                    "Configuration field 'time_limit_seconds' must be a non-negative integer");
            }
            if (*parsed == 0) {
                result.time_limit_seconds.reset();
            } else {
                result.time_limit_seconds = *parsed;
            }
        } else if (key == "dry_run") {
            result.dry_run = truthy(value);
        } else if (key == "verbose") {
            result.verbose = truthy(value);
        } else if (key == "log_file") {
            auto str = value_to_string(value);
            if (!str) {
                throw std::runtime_error("Configuration field 'log_file' must be a string");
            }
            fs::path resolved = resolve_config_path(base_dir, *str);
            result.log_file = resolved.string();
        }
    }
    return result;
}

std::vector<ConfiguredRun> load_config(const Options& base, const fs::path& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open configuration file: " + path.string());
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    auto json_value = simple_json::parse(buffer.str());
    std::vector<ConfiguredRun> runs;
    auto process_entry = [&](const simple_json::JsonValue& value) {
        if (!value.is_object()) {
            throw std::runtime_error("Each run definition must be a JSON object");
        }
        std::optional<std::string> label;
        Options opts = apply_config_entry(base, value.as_object(), path.parent_path(), label);
        runs.push_back(ConfiguredRun{opts, label});
    };
    if (json_value.is_object()) {
        const auto& obj = json_value.as_object();
        auto it = obj.find("runs");
        if (it != obj.end()) {
            if (!it->second.is_array()) {
                throw std::runtime_error("Configuration 'runs' field must be an array");
            }
            for (const auto& entry : it->second.as_array()) {
                process_entry(entry);
            }
        } else {
            process_entry(json_value);
        }
    } else if (json_value.is_array()) {
        for (const auto& entry : json_value.as_array()) {
            process_entry(entry);
        }
    } else {
        throw std::runtime_error("Configuration must be a JSON object or array");
    }
    if (runs.empty()) {
        throw std::runtime_error("Configuration file did not contain any runs");
    }
    return runs;
}

}  // namespace

int main(int argc, char** argv) {
    Options opts = parse_arguments(argc, argv);
    if (opts.config_path) {
        fs::path config(*opts.config_path);
        try {
            auto runs = load_config(opts, config);
            int exit_code = EXIT_SUCCESS;
            for (std::size_t i = 0; i < runs.size(); ++i) {
                const auto& run = runs[i];
                std::string label = run.label.value_or("configuration #" + std::to_string(i + 1));
                std::cout << "\n=== Running " << label << " ===\n";
                exit_code = execute_single_run(run.options, run.label);
                if (exit_code != EXIT_SUCCESS) {
                    return exit_code;
                }
            }
            return exit_code;
        } catch (const std::exception& ex) {
            std::cerr << "Error: " << ex.what() << "\n";
            return EXIT_FAILURE;
        }
    }
    return execute_single_run(opts, std::nullopt);
}
