

static LazySolveResult lazy_solve_with_waiting_time(
    CurrentSolution& current_solution,
    const std::vector<std::vector<char>>& map,
    const std::vector<std::vector<char>>& masked_map,                           
    const std::set<std::pair<int,int>>& zone_positions_set,
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    int start_t, int end_t, 
    int offset,
    int initial_waiting_time_amount = 0) {

    LazySolveResult lazy_result;
    lazy_result.solution_found = false;
    lazy_result.local_paths = local_zone_paths;
    lazy_result.discovered_vertex_collisions = initial_vertex_collisions;
    lazy_result.discovered_edge_collisions = initial_edge_collisions;
    

    //Step 1: assert waiting time

    //zone positions
    //Step 2: create the local problem
    LocalZoneState local_zone_state = build_local_problem_for_zone(
        current_solution,
        zone_positions_set,
        masked_map, 
        map, 
        conflict_map, 
        conflict_meta,
        offset, start_t, end_t);
    std::cout << "[Waiting_time_Solve] Local zone state: " << local_zone_state.local_zone_paths.size() << " agents" << std::endl;
    std::cout << "[Waiting_time_Solve] number of pseudo agents: " << local_zone_state.original_agent_id.size() << std::endl;



//handle pseudo agents
//if fail restore waiting time




    return lazy_result;
}