


std::pair<std::set<int>, bool> choose_agents_to_use_waiting_time(
    const std::vector<ConflictMeta>& current_conflicts, 
    const CurrentSolution& current_solution) {

    std::set<int> agents_to_use_waiting_time;
    bool can_use_waiting_time = true;
    //iterate through all current conflicts -> skip if one of the agents is already chosen, otherwise add the one with more waiting time
    for (const auto& conflict : current_conflicts) {
        int agent_1 = conflict.agent1;
        int agent_2 = conflict.agent2;
        int agent_1_waiting_time = current_solution.get_waiting_time(agent_1);
        int agent_2_waiting_time = current_solution.get_waiting_time(agent_2);
        //does one have waiting time?
        if (agent_1_waiting_time > 0 || agent_2_waiting_time > 0) {
            //if one of the agent is alerady chosen, skip
            if (agents_to_use_waiting_time.count(agent_1) > 0 || agents_to_use_waiting_time.count(agent_2) > 0) {
                continue;
            }
            //add the one with more waiting time
            if (agent_1_waiting_time >= agent_2_waiting_time) {
                agents_to_use_waiting_time.insert(agent_1);
            } else {
                agents_to_use_waiting_time.insert(agent_2);
            }
        }else{
            //if both agents have no waiting time, we can't use waiting time
            can_use_waiting_time = false;
            break; // no need to check other agents
        }
    }
    return {agents_to_use_waiting_time, can_use_waiting_time};
}


LazySolveResult lazy_result;
    lazy_result.solution_found = false;
    lazy_result.local_paths = local_zone_paths;
    lazy_result.discovered_vertex_collisions = initial_vertex_collisions;
    lazy_result.discovered_edge_collisions = initial_edge_collisions;

LazySolveResult lazy_solve_with_waiting_time(
    CurrentSolution& current_solution,
    const std::vector<std::vector<char>>& map,
    const std::vector<std::vector<char>>& masked_map,                           
    const std::set<std::pair<int,int>>& local_zone_positions,
    const std::vector<ConflictMeta>& conflict_meta,
    const std::vector<int>& local_zone_conflict_indices,
    const std::vector<std::vector<std::vector<int>>>& conflict_map,
    int start_t, int end_t, 
    int offset,
    int initial_waiting_time_amount = 0) {

    
    

    //Step 1: assert waiting time and make a backup so we can restore it if waiting time solve fails
    auto waiting_time_backup = current_solution.backup_waiting_times();

    int using_waiting_time = initial_waiting_time_amount;

    std::vector<ConflictMeta> current_conflicts;
    for (int conflict_idx : local_zone_conflict_indices) {
        if (conflict_idx >= 0 && conflict_idx < (int)conflict_meta.size()) {
            current_conflicts.push_back(conflict_meta[conflict_idx]);
        }
    }
    
    //Step 2: choose agents to use waiting time and check if we can use waiting time
    auto [agents_to_use_waiting_time, can_use_waiting_time] = choose_agents_to_use_waiting_time(current_conflicts, current_solution);

    //initialize start and end time
    const int zone_start_t = start_t;
    int old_end_t = end_t;
    int new_end_t = end_t;

    bool solution_found = false;
    while(!solution_found && can_use_waiting_time){
        
        //zone positions
        //Step 2: create the local problem
        LocalZoneState local_zone_state = build_local_problem_for_zone(
            current_solution,
            local_zone_positions,
            masked_map, 
            map, 
            conflict_map, 
            conflict_meta,
            offset, start_t, end_t);


        std::vector<std::pair<int,int>> local_zone_paths;

        std::unordered_map<int, std::pair<int,int>> local_entry_exit_time;
        std::unordered_map<int, std::shared_ptr<MDD>> local_mdds;
        std::unordered_map<int, int> original_agent_id; //remember original agent id for pseudo agents (fake id, real id)

        std::cout << "[Waiting_time_Solve] Local zone state: " << local_zone_state.local_zone_paths.size() << " agents" << std::endl;
        std::cout << "[Waiting_time_Solve] number of pseudo agents: " << local_zone_state.original_agent_id.size() << std::endl;

        //update agents_to_use_waiting_time
        [agents_to_use_waiting_time, can_use_waiting_time] = choose_agents_to_use_waiting_time(current_conflicts, current_solution);

    //handle pseudo agents
    //if fail restore waiting time



    }
    return lazy_result;
}