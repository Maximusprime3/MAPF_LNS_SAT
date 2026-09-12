#include "lnssat/LNS.h"
#include "lnssat/Load_LNSProblem.h"
#include "lnssat/SolutionVerifier.h"
#include <iostream>
int main(int argc,char** argv) {
    if(argc != 7) return 64;
    SolveRequest request{argv[1],argv[2],std::stoi(argv[3]),std::stoi(argv[4])};
    SolverConfig config; config.seed=std::stoi(argv[5]);config.neighborhood_variant=*parse_neighborhood_variant(argv[6]);config.log_level=LogLevel::Quiet;
    auto r=LNS(request,config);
    auto p=load_problem(request.map_path,request.scenario_path,request.num_agents,request.scenario_index);
    std::cout << "CONTRACT " << solve_status_name(r.status) << ' ' << r.makespan << ' ' << (r.solved()&&p&&mapf::verify_solution(r.paths,p->starts,p->goals,p->grid).valid()) << '\n';
    if(r.solved()) for(int id=0;id<request.num_agents;++id) {std::cout<<"PATH "<<id;for(auto xy:r.paths.at(id)) std::cout<<' '<<xy.first<<','<<xy.second;std::cout<<'\n';}
}
