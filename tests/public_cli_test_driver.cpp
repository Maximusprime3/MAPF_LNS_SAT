// Test-only executable: no fault-control environment hooks in the shipped CLI.
#include "lnssat/PublicResult.h"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <thread>
static LNSResult fake(const SolveRequest& request, const SolverConfig& config,
                      const LNSProblem& problem, SolverDeadline deadline) {
    const std::string mode = std::getenv("LNS_TEST_MODE") ? std::getenv("LNS_TEST_MODE") : "throw";
    if (mode == "throw") throw std::invalid_argument("controlled backend failure");
    if (mode == "snapshot") {
        std::ofstream map(request.map_path,std::ios::binary); map << "changed after loading\n"; map.close();
        return LNS(request,config,problem,deadline);
    }
    if (mode == "late") {
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        LNSResult r; r.status=SolveStatus::Exhausted; r.message="controlled timeout";r.termination_reason="wall_clock_limit";return r;
    }
    LNSResult result; result.status = SolveStatus::Solved;
    result.paths[0] = {{-1,-1}}; result.message = "controlled corrupt solution";
    return result;
}
int main(int argc,char** argv) { return lnssat::public_cli(argc,argv,fake); }
