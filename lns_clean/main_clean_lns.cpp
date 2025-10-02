#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Forward declaration from the clean LNS implementation.
std::unordered_map<int, std::vector<std::pair<int, int>>> LNS(
    const std::string& map_path,
    const std::string& scenario_path,
    int num_agents,
    int scenario_index,
    bool use_minisat,
    int seed);

namespace {

struct Config {
    std::string map_path;
    std::string scenario_path;
    int num_agents = -1;
    int scenario_index = 0;
    std::string solver = "minisat";
    int seed = 42;
    std::string log_path;
};

std::string trim(const std::string& input) {
    const auto begin = std::find_if_not(input.begin(), input.end(), [](unsigned char ch) {
        return std::isspace(ch);
    });
    const auto end = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char ch) {
        return std::isspace(ch);
    }).base();
    if (begin >= end) {
        return "";
    }
    return std::string(begin, end);
}

bool load_config(const std::string& path, Config& config) {
    std::ifstream in(path);
    if (!in.is_open()) {
        std::cerr << "Failed to open config file: " << path << std::endl;
        return false;
    }

    std::unordered_map<std::string, std::string> kv;
    std::string line;
    size_t line_number = 0;
    while (std::getline(in, line)) {
        ++line_number;
        line = trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const auto delimiter_pos = line.find('=');
        if (delimiter_pos == std::string::npos) {
            std::cerr << "Invalid config line " << line_number << ": " << line << std::endl;
            return false;
        }
        std::string key = trim(line.substr(0, delimiter_pos));
        std::string value = trim(line.substr(delimiter_pos + 1));
        if (key.empty()) {
            std::cerr << "Empty key on line " << line_number << std::endl;
            return false;
        }
        kv[key] = value;
    }

    auto get = [&kv](const std::string& key) -> std::string {
        auto it = kv.find(key);
        if (it == kv.end()) {
            return "";
        }
        return it->second;
    };

    if (!get("map").empty()) {
        config.map_path = get("map");
    }
    if (!get("scenario").empty()) {
        config.scenario_path = get("scenario");
    }
    if (!get("num_agents").empty()) {
        config.num_agents = std::stoi(get("num_agents"));
    }
    if (!get("scenario_index").empty()) {
        config.scenario_index = std::stoi(get("scenario_index"));
    }
    if (!get("solver").empty()) {
        config.solver = get("solver");
    }
    if (!get("seed").empty()) {
        config.seed = std::stoi(get("seed"));
    }
    if (!get("log").empty()) {
        config.log_path = get("log");
    }

    if (config.map_path.empty()) {
        std::cerr << "Config is missing required field: map" << std::endl;
        return false;
    }
    if (config.scenario_path.empty()) {
        std::cerr << "Config is missing required field: scenario" << std::endl;
        return false;
    }
    if (config.num_agents <= 0) {
        std::cerr << "Config field num_agents must be positive" << std::endl;
        return false;
    }

    return true;
}

void print_usage(const char* executable) {
    std::cerr << "Usage: " << executable
              << " <map_path> <scenario_path> <num_agents> <scenario_index> <solver> [seed]" << std::endl
              << "   or: " << executable << " --config <config_file>" << std::endl
              << "  solver: minisat | probsat" << std::endl
              << "  example: ./main_clean_lns mapf-map/maze-32-32-2.map mapf-scen-even/scen-even/maze-32-32-2-even-1.scen 30 minisat 42" << std::endl;
}

std::string to_lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

}  // namespace

int main(int argc, char** argv) {
    std::string map_path;
    std::string scenario_path;
    int num_agents = -1;
    int scenario_index = 0;
    std::string solver = "minisat";
    int seed = 42;
    std::string log_path;

    if (argc >= 3 && std::string(argv[1]) == "--config") {
        Config config;
        if (!load_config(argv[2], config)) {
            return 2;
        }
        map_path = config.map_path;
        scenario_path = config.scenario_path;
        num_agents = config.num_agents;
        scenario_index = config.scenario_index;
        solver = config.solver;
        seed = config.seed;
        log_path = config.log_path;
    } else {
        if (argc < 6) {
            print_usage(argv[0]);
            return 2;
        }
        map_path = argv[1];
        scenario_path = argv[2];
        num_agents = std::atoi(argv[3]);
        scenario_index = std::atoi(argv[4]);
        solver = argv[5];
        seed = (argc >= 7) ? std::atoi(argv[6]) : 42;
    }

    if (num_agents <= 0) {
        std::cerr << "Number of agents must be positive" << std::endl;
        return 2;
    }

    bool use_minisat = (to_lower(solver) == "minisat");

    std::unique_ptr<std::ofstream> log_stream;
    std::streambuf* original_cout_buf = nullptr;
    if (!log_path.empty()) {
        log_stream = std::make_unique<std::ofstream>(log_path);
        if (!log_stream->is_open()) {
            std::cerr << "Failed to open log file: " << log_path << std::endl;
            return 2;
        }
        // Redirect cout to the log file and keep the original buffer to restore later.
        original_cout_buf = std::cout.rdbuf(log_stream->rdbuf());
    }

    auto result = LNS(map_path, scenario_path, num_agents, scenario_index, use_minisat, seed);
    if (result.empty()) {
        std::cerr << "LNS did not return any agent paths." << std::endl;
        if (original_cout_buf != nullptr) {
            std::cout.flush();
            std::cout.rdbuf(original_cout_buf);
        }
        return 1;
    }

    if (original_cout_buf != nullptr) {
        std::cout.flush();
        std::cout.rdbuf(original_cout_buf);
    }
    return 0;
}