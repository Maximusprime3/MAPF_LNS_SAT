#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <filesystem>
#include <cstdlib>
#include <chrono>
#include <ctime>

static std::unordered_map<std::string, std::string> parse_args(int argc, char** argv) {
    std::unordered_map<std::string, std::string> args;
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        if (key.rfind("--", 0) == 0) {
            if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
                args[key.substr(2)] = argv[++i];
            } else {
                args[key.substr(2)] = "1"; // flag-like
            }
        }
    }
    return args;
}

static int count_scenario_rows(const std::string& scenario_path) {
    std::ifstream in(scenario_path);
    if (!in.is_open()) return -1;
    std::string line;
    bool first = true;
    int count = 0;
    while (std::getline(in, line)) {
        if (first) { first = false; continue; } // skip version/header
        if (!line.empty()) ++count;
    }
    return count;
}

int main(int argc, char** argv) {
    auto args = parse_args(argc, argv);

    const std::string solver = args.count("solver") ? args["solver"] : std::string("minisat");
    const std::string map_path = args.count("map") ? args["map"] : std::string("mapf-map/empty-16-16.map");
    const std::string scenario_path = args.count("scenario") ? args["scenario"] : std::string("mapf-scen-even/scen-even/empty-16-16-even-1.scen");
    const int num_agents = args.count("num_agents") ? std::stoi(args["num_agents"]) : 3;
    const int num_runs = args.count("num_runs") ? std::stoi(args["num_runs"]) : 1;
    const int start_offset = args.count("start_offset") ? std::stoi(args["start_offset"]) : 0;
    const int initial_timesteps = args.count("initial_timesteps") ? std::stoi(args["initial_timesteps"]) : 2;
    const int max_increase = args.count("max_increase") ? std::stoi(args["max_increase"]) : 10;
    const long long seed = args.count("seed") ? std::stoll(args["seed"]) : 42LL;
    const long long max_runs = args.count("max_runs") ? std::stoll(args["max_runs"]) : 1LL;       // ProbSAT only
    const long long max_flips = args.count("max_flips") ? std::stoll(args["max_flips"]) : 50000LL; // ProbSAT only

    // Validate scenario capacity
    int total_rows = count_scenario_rows(scenario_path);
    if (total_rows < 0) {
        std::cerr << "ERROR: Could not open scenario file: " << scenario_path << std::endl;
        return 1;
    }
    long long required = static_cast<long long>(start_offset) + static_cast<long long>(num_runs) * static_cast<long long>(num_agents);
    if (required > total_rows) {
        std::cerr << "ERROR: Scenario rows insufficient. total_rows=" << total_rows
                  << ", required=" << required << " (start_offset + num_runs*num_agents)" << std::endl;
        return 1;
    }

    // Ensure data directory exists
    try {
        if (!std::filesystem::create_directories("data") && !std::filesystem::exists("data")) {
            std::cerr << "ERROR: Could not create or access 'data' directory for logging." << std::endl;
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Exception while creating 'data' directory: " << e.what() << std::endl;
        return 1;
    }

    // Choose executable
    const std::string minisat_exec = "./test_satsolvermanager_minisat";
    const std::string probsat_exec = "./test_satsolvermanager_probsat";
    const bool use_minisat = (solver == "minisat" || solver == "MiniSAT");

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    char buf[64];
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &now_c);
#else
    gmtime_r(&now_c, &tm);
#endif
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
    std::string run_start_iso(buf);

    for (int run_id = 0; run_id < num_runs; ++run_id) {
        int scenario_offset = start_offset + run_id * num_agents;
        std::ostringstream cmd;
        // Prefix environment for the child process (portable via shell)
        cmd << "RUN_ID=" << run_id
            << " RUN_START_ISO=" << '"' << run_start_iso << '"'
            << " SCENARIO_OFFSET=" << scenario_offset << ' ';
        if (use_minisat) {
            cmd << minisat_exec
                << " --map " << '"' << map_path << '"'
                << " --scenario " << '"' << scenario_path << '"'
                << " --num_agents " << num_agents
                << " --initial_timesteps " << initial_timesteps
                << " --max_increase " << max_increase
                << " --seed " << seed
                << " --scenario_offset " << scenario_offset
                << " --run_id " << run_id;
        } else {
            cmd << probsat_exec
                << " --map " << '"' << map_path << '"'
                << " --scenario " << '"' << scenario_path << '"'
                << " --num_agents " << num_agents
                << " --initial_timesteps " << initial_timesteps
                << " --max_increase " << max_increase
                << " --seed " << seed
                << " --scenario_offset " << scenario_offset
                << " --run_id " << run_id
                << " --max_runs " << max_runs
                << " --max_flips " << max_flips;
        }
        // (SCENARIO_OFFSET already exported before the executable)

        std::cout << "[RUN] (" << run_id << ") scenario_offset=" << scenario_offset << " => executing: " << cmd.str() << std::endl;
        int rc = std::system(cmd.str().c_str());
        if (rc != 0) {
            std::cerr << "ERROR: Run " << run_id << " failed with code " << rc << std::endl;
            return rc;
        }
    }

    std::cout << "All runs completed successfully." << std::endl;
    return 0;
}



