#include <iostream>
#include <string>
#include <cstdlib>

// Forward-declare from LNSSolver.cpp
int run_crude_lns(const std::string& map_path,
                  const std::string& scenario_path,
                  int num_agents,
                  int scenario_index,
                  bool use_minisat,
                  int seed);

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "Usage: " << argv[0]
                  << " <map_path> <scenario_path> <num_agents> <scenario_index> <solver> [seed]\n"
                  << "  solver: minisat | probsat\n"
                  << "  example: ./main_crude_lns mapf-map/maze-32-32-2.map mapf-scen-even/scen-even/maze-32-32-2-even-1.scen 3 0 minisat 42\n";
        return 2;
    }
    std::string map_path = argv[1];
    std::string scenario_path = argv[2];
    int num_agents = std::atoi(argv[3]);
    int scenario_index = std::atoi(argv[4]);
    std::string solver = argv[5];
    int seed = (argc >= 7) ? std::atoi(argv[6]) : 42;

    bool use_minisat = (solver == "minisat");

    return run_crude_lns(map_path, scenario_path, num_agents, scenario_index, use_minisat, seed);
}

