#ifndef LNS_LAZY_SAT_SOLVE_H
#define LNS_LAZY_SAT_SOLVE_H

#include "CNFConstructor.h"
#include "CNF.h"
#include <unordered_map>
#include <vector>
#include <tuple>
#include <set>
#include <utility>


// Result type for lazy SAT local-zone solving
struct LazySolveResult {
    bool solution_found;
    std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths;
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> discovered_vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> discovered_edge_collisions;
    // latest collisions that were discovered before UNSAT
    std::vector<std::tuple<int, int, std::pair<int,int>, int>> latest_discovered_vertex_collisions;
    std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>> latest_discovered_edge_collisions;
};

// Solves a local zone using lazy SAT by iteratively adding collision-prevention clauses
LazySolveResult lazy_SAT_solve(
    CNF& local_cnf,
    CNFConstructor& cnf_constructor,
    const std::unordered_map<int, std::pair<int,int>>& local_entry_exit_time,
    int start_t, int end_t,
    int max_iterations = 1000,
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& initial_vertex_collisions = {},
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& initial_edge_collisions = {}
);

#endif // LNS_LAZY_SAT_SOLVE_H


