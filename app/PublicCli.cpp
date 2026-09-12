#include "lnssat/PublicResult.h"
#include "lnssat/Logging.h"
#include "lnssat/Sha256.h"
#include "lnssat/SolutionVerifier.h"
#include "BuildProvenance.h"
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <fcntl.h>
#include <unistd.h>

namespace lnssat {
namespace {
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
struct OutputError : std::runtime_error { using std::runtime_error::runtime_error; };
struct Arguments {
    SolveRequest request;
    SolverConfig config;
    std::string command = "solve", output, solution, configuration_file;
};
const char* help = R"(LNS-SAT public CLI (result schema 1)
Usage:
  lns-sat solve --map MAP --scenario SCEN --agents N [options]
  lns-sat verify --map MAP --scenario SCEN --solution FILE [--log-level LEVEL]
  lns-sat --help
  lns-sat --version
Solve options (defaults):
  --config FILE                      flat key=value configuration, CLI overrides
  --scenario-index N                 block index, first row = N * agents (0)
  --seed N                           signed 32-bit seed (42)
  --variant NAME                     lns-sat | initial-radius-2 | fixed-step-2 | increasing-step
  --makespan-increment N              positive additive step (1)
  --makespan-increase-limit N         maximum additive increase, not count (10)
  --makespan-bound N                  absolute bound in moves (unlimited)
  --lazy-iteration-limit N            positive iteration cap (10000000)
  --full-map-fallback-threshold X     fraction in (0,1] (0.95)
  --wall-clock-limit-ms N             positive cooperative budget in milliseconds
  --log-level LEVEL                   quiet | info | debug (info)
  --output FILE                      overwrite result at explicit destination
  --format json                      schema 1 JSON (only supported format)
Legacy: MAP SCEN AGENTS INDEX [SEED [VARIANT]], or --config FILE.
Relative paths, including INI paths, use the working directory. Duplicate and
unknown options/keys are errors. No implicit output files. Diagnostics use stderr.
Solve exit codes: 0 solved, 1 exhausted, 2 invalid input, 3 internal failure.
Verify: 0 valid, 2 malformed/unreadable input, 4 invalid submitted solution.
Either: 5 output failure. Exhaustion is not proof of global unsatisfiability.
See docs/CLI.md and docs/RESULT_SCHEMA.md for full contracts.
)";
std::string absolute(const std::string& path) {
    if (path.empty() || path.find('\0') != std::string::npos || !lnssat_json::valid_utf8(path)) throw std::invalid_argument("Expected nonempty UTF-8 path without NUL");
    return fs::absolute(path).lexically_normal().string();
}
std::string read_bytes(const std::string& path) {
    if (!fs::is_regular_file(path)) throw std::invalid_argument("Input is not a readable regular file: " + path);
    std::ifstream in(path,std::ios::binary);
    if (!in) throw std::invalid_argument("Cannot open input: " + path);
    std::string bytes;
    char buffer[65536];
    while (in) {
        in.read(buffer,sizeof buffer);
        bytes.append(buffer,static_cast<size_t>(in.gcount()));
        if (bytes.size() > 256u * 1024u * 1024u) throw std::invalid_argument("Input exceeds 256 MiB: " + path);
    }
    if (!in.eof() || in.bad()) throw std::invalid_argument("Failed reading input: " + path);
    return bytes;
}
bool same_file(const std::string& a, const std::string& b) {
    if (a.empty() || b.empty()) return false;
    if (fs::weakly_canonical(a) == fs::weakly_canonical(b)) return true;
    return fs::exists(a) && fs::exists(b) && fs::equivalent(a,b);
}
void check_destination(const std::string& output, const std::vector<std::string>& inputs) {
    if (output.empty()) return;
    for (const auto& input : inputs) if (same_file(output,input)) throw OutputError("Output aliases an input or another output: " + output);
    const auto status = fs::symlink_status(output);
    if (fs::is_symlink(status) || (fs::exists(status) && !fs::is_regular_file(status))) throw OutputError("Output must be a regular file, not a symlink: " + output);
    if (!fs::is_directory(fs::path(output).parent_path())) throw OutputError("Output parent directory does not exist: " + output);
}
class AtomicOutput {
    std::string destination_, temporary_;
    int fd_ = -1;
public:
    explicit AtomicOutput(const std::string& path) : destination_(path) {
        std::string pattern = path + ".tmp.XXXXXX";
        std::vector<char> name(pattern.begin(),pattern.end()); name.push_back('\0');
        fd_ = mkstemp(name.data());
        if (fd_ < 0) throw OutputError("Cannot create output: " + path + ": " + std::strerror(errno));
        temporary_ = name.data();
        if (::unlink(path.c_str()) != 0 && errno != ENOENT) {
            ::close(fd_); fd_ = -1; ::unlink(temporary_.c_str());
            throw OutputError("Cannot invalidate previous output: " + path);
        }
    }
    ~AtomicOutput() { if (fd_ >= 0) ::close(fd_); if (!temporary_.empty()) ::unlink(temporary_.c_str()); }
    AtomicOutput(const AtomicOutput&) = delete;
    AtomicOutput& operator=(const AtomicOutput&) = delete;
    void publish(const std::string& bytes) {
        size_t offset = 0;
        while (offset < bytes.size()) {
            const auto n = ::write(fd_,bytes.data()+offset,bytes.size()-offset);
            if (n < 0 && errno == EINTR) continue;
            if (n <= 0) throw OutputError("Failed writing output: " + destination_);
            offset += static_cast<size_t>(n);
        }
        if (::fsync(fd_) != 0) throw OutputError("Failed flushing output: " + destination_);
        const int fd = fd_; fd_ = -1;
        if (::close(fd) != 0) throw OutputError("Failed closing output: " + destination_);
        if (::rename(temporary_.c_str(),destination_.c_str()) != 0) throw OutputError("Failed publishing output: " + destination_);
        temporary_.clear();
    }
};
Arguments parse(int argc, char** argv) {
    if (argc < 2) throw std::invalid_argument("Expected solve, verify, --help, or --version");
    Arguments args;
    int start = 1;
    const std::string first = argv[1];
    if (first == "solve" || first == "verify") { args.command = first; start = 2; }
    else if (first != "--config") {
        std::vector<std::string> positional(argv+1,argv+argc);
        if (first.rfind("-",0) == 0) throw std::invalid_argument("Unknown option: " + first);
        auto resolution = resolve_positional_solve_arguments(positional);
        if (!resolution.valid()) throw std::invalid_argument(resolution.validation.message);
        args.request = resolution.request; args.config = resolution.config;
        return args;
    }
    const std::map<std::string,std::string> names{
        {"--map","map"},{"--scenario","scenario"},{"--agents","num_agents"},
        {"--scenario-index","scenario_index"},{"--seed","seed"},{"--variant","variant"},
        {"--makespan-increment","makespan_increment"},{"--makespan-increase-limit","makespan_increase_limit"},
        {"--makespan-bound","makespan_bound"},{"--lazy-iteration-limit","lazy_iteration_limit"},
        {"--full-map-fallback-threshold","full_map_fallback_threshold"},{"--wall-clock-limit-ms","wall_clock_limit_ms"},
        {"--log-level","log_level"}};
    std::map<std::string,std::string> options;
    for (int i = start; i < argc; ++i) {
        const std::string key = argv[i];
        const bool special = key == "--config" || key == "--output" || key == "--format" || key == "--solution";
        if (!special && !names.count(key)) throw std::invalid_argument("Unknown option: " + key);
        if (args.command == "verify" && key != "--map" && key != "--scenario" && key != "--solution" && key != "--log-level") throw std::invalid_argument("Unsupported verify option: " + key);
        if (args.command == "solve" && key == "--solution") throw std::invalid_argument("--solution is a verify option");
        if (i+1 == argc || std::string(argv[i+1]).empty() || std::string(argv[i+1]).rfind("--",0) == 0) throw std::invalid_argument("Missing value for " + key);
        if (!options.emplace(key,argv[++i]).second) throw std::invalid_argument("Duplicate option: " + key);
    }
    if (options.count("--config")) {
        args.configuration_file = absolute(options.at("--config"));
        auto resolution = load_solve_configuration_file(args.configuration_file);
        if (!resolution.valid()) throw std::invalid_argument(resolution.validation.message);
        args.request = resolution.request; args.config = resolution.config;
    }
    for (const auto& option : options) {
        if (option.first == "--config") continue;
        if (option.first == "--output") args.output = absolute(option.second);
        else if (option.first == "--solution") args.solution = absolute(option.second);
        else if (option.first == "--format") { if (option.second != "json") throw std::invalid_argument("Unsupported format: " + option.second); }
        else if (auto error = apply_solver_configuration_value(names.at(option.first),option.second,args.request,args.config)) throw std::invalid_argument(*error);
    }
    if (args.command == "solve") {
        auto validation = validate_solver_configuration(args.request,args.config);
        if (!validation.valid) throw std::invalid_argument(validation.message);
    } else if (args.request.map_path.empty() || args.request.scenario_path.empty() || args.solution.empty()) throw std::invalid_argument("verify requires --map, --scenario and --solution");
    return args;
}
InputSnapshot snapshot(const SolveRequest& request, std::string& error) {
    InputSnapshot result;
    // Attempt both reads, retaining available provenance if either input fails.
    try { result.map_bytes = read_bytes(request.map_path); result.map_hash = sha256(result.map_bytes); }
    catch (const std::exception& e) { error = e.what(); }
    try { result.scenario_bytes = read_bytes(request.scenario_path); result.scenario_hash = sha256(result.scenario_bytes); }
    catch (const std::exception& e) { if (!error.empty()) error += "; "; error += e.what(); }
    return result;
}
std::optional<LNSProblem> problem(const InputSnapshot& inputs, const SolveRequest& request) {
    std::istringstream map(inputs.map_bytes), scenario(inputs.scenario_bytes);
    return load_problem(map,scenario,request.num_agents,request.scenario_index);
}
int verify(const Arguments& args) {
    const auto submitted = read_result_document(lnssat_json::parse(read_bytes(args.solution)));
    std::string error;
    auto inputs = snapshot(args.request,error);
    if (!error.empty()) throw std::invalid_argument(error);
    if (!submitted.has_solution) { std::cerr << "VERIFY_RESULT invalid_solution: document contains no successful solution\n"; return 4; }
    if (!submitted.selection_consistent) { std::cerr << "VERIFY_RESULT invalid_solution: inconsistent scenario selection\n"; return 4; }
    if (inputs.map_hash.as_string() != submitted.map_hash || inputs.scenario_hash.as_string() != submitted.scenario_hash) {
        std::cerr << "VERIFY_RESULT invalid_solution: input checksum mismatch\n"; return 4;
    }
    auto loaded = problem(inputs,submitted.request);
    if (!loaded) throw std::invalid_argument("Cannot load selected map/scenario instance");
    const auto report = mapf::verify_solution(submitted.paths,loaded->starts,loaded->goals,loaded->grid);
    for (const auto& issue : report.issues) std::cerr << "VERIFY_RESULT " << mapf::verification_issue_code_name(issue.code) << ": " << issue.message << '\n';
    if (!report.valid()) return 4;
    if (report.makespan != submitted.makespan) { std::cerr << "VERIFY_RESULT invalid_solution: recorded makespan differs from recomputed makespan\n"; return 4; }
    info_log() << "VERIFY_RESULT valid agents=" << submitted.request.num_agents << " makespan=" << report.makespan << '\n';
    return 0;
}
}

int public_cli(int argc, char** argv, SolveFunction solve) {
    const auto start = Clock::now();
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") { std::cout << help; std::cout.flush(); return std::cout ? 0 : 5; }
        if (argc == 2 && std::string(argv[1]) == "--version") {
            std::cout << "lns-sat schema=1 " << LNS_BUILD_JSON << '\n'; std::cout.flush(); return std::cout ? 0 : 5;
        }
        auto args = parse(argc,argv);
        args.request.map_path = absolute(args.request.map_path);
        args.request.scenario_path = absolute(args.request.scenario_path);
        if (!args.config.log_output_path.empty()) args.config.log_output_path = absolute(args.config.log_output_path);
        LogScope logging(args.config.log_level);
        if (args.command == "verify") return verify(args);
        const std::vector<std::string> protected_inputs{args.request.map_path,args.request.scenario_path,args.configuration_file};
        try {
            check_destination(args.output,protected_inputs);
            auto log_protection = protected_inputs; log_protection.push_back(args.output);
            check_destination(args.config.log_output_path,log_protection);
        } catch (const fs::filesystem_error& e) { throw OutputError(e.what()); }
        std::unique_ptr<AtomicOutput> output, diagnostic_output;
        if (!args.output.empty()) output = std::make_unique<AtomicOutput>(args.output);
        if (!args.config.log_output_path.empty()) diagnostic_output = std::make_unique<AtomicOutput>(args.config.log_output_path);
        std::ostringstream diagnostic_text;
        LogScope diagnostic_scope(args.config.log_level,diagnostic_output ? &diagnostic_text : &std::cerr);
        const auto deadline = args.config.wall_clock_limit ? SolverDeadline(start + *args.config.wall_clock_limit) : SolverDeadline{};
        LNSResult result;
        result.seed = args.config.seed; result.neighborhood_variant = args.config.neighborhood_variant;
        Json solver_ms;
        std::string error;
        const auto inputs = snapshot(args.request,error);
        try {
            if (!error.empty()) throw std::invalid_argument(error);
            const auto loaded = problem(inputs,args.request);
            if (!loaded) throw std::invalid_argument("Failed to load selected map/scenario instance");
            const auto solver_start = Clock::now();
            try { result = solve(args.request,args.config,*loaded,deadline); }
            catch (const std::exception& e) { solver_ms = std::chrono::duration<double,std::milli>(Clock::now()-solver_start).count(); throw std::runtime_error(std::string("Solver exception: ") + e.what()); }
            catch (...) { solver_ms = std::chrono::duration<double,std::milli>(Clock::now()-solver_start).count(); throw; }
            solver_ms = std::chrono::duration<double,std::milli>(Clock::now()-solver_start).count();
            if (result.solved()) {
                const auto report = mapf::verify_solution(result.paths,loaded->starts,loaded->goals,loaded->grid);
                result.verification = report.valid() ? "passed" : "failed";
                if (!report.valid()) {
                    result.status = SolveStatus::InvalidState; result.termination_reason = "internal_failure";
                    result.message = "Public result rejected by independent verification: " + report.issues.front().message;
                } else {
                    result.makespan = report.makespan;
                    result.termination_reason = "solved";
                    if (solver_deadline_reached(deadline)) {
                        result.status = SolveStatus::Exhausted; result.termination_reason = "wall_clock_limit";
                        result.message = "Wall-clock limit reached before publishing verified result";
                    }
                }
            }
        } catch (const std::invalid_argument& e) {
            result.status = SolveStatus::InvalidInput; result.termination_reason = "invalid_input"; result.message = e.what();
        } catch (const std::exception& e) {
            result.status = SolveStatus::InvalidState; result.termination_reason = "internal_failure"; result.message = e.what();
        } catch (...) {
            result.status = SolveStatus::InvalidState; result.termination_reason = "internal_failure"; result.message = "Unknown internal failure";
        }
        if (!result.solved()) { result.paths.clear(); result.makespan = -1; }
        const double total_ms = std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        if (!result.solved()) std::cerr << "LNS_RESULT status=" << solve_status_name(result.status) << " reason=" << result.termination_reason << " message=" << result.message << '\n';
        else info_log() << "LNS_RESULT status=solved makespan=" << result.makespan << '\n';
        if (diagnostic_output) diagnostic_output->publish(diagnostic_text.str());
        if (output) output->publish(lnssat_json::dump(result_document(args.request,args.config,result,inputs,total_ms,solver_ms,lnssat_json::parse(LNS_BUILD_JSON))));
        return solve_status_exit_code(result.status);
    } catch (const OutputError& e) {
        std::cerr << "OUTPUT_ERROR " << e.what() << '\n'; return 5;
    } catch (const fs::filesystem_error& e) {
        std::cerr << "INPUT_OR_OUTPUT_ERROR " << e.what() << '\n'; return 2;
    } catch (const std::exception& e) {
        std::cerr << "INVALID_INPUT " << e.what() << '\n'; return 2;
    }
}
}
