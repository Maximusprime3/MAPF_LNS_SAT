


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

    
    

    //assert waiting time and make a backup so we can restore it if waiting time solve fails
    auto waiting_time_backup = current_solution.backup_waiting_times();

    int using_waiting_time = initial_waiting_time_amount; 

    std::vector<ConflictMeta> current_conflicts;
    for (int conflict_idx : local_zone_conflict_indices) {
        if (conflict_idx >= 0 && conflict_idx < (int)conflict_meta.size()) {
            current_conflicts.push_back(conflict_meta[conflict_idx]);
        }
    }
    
    //Choose agents from initial conflicts to use waiting time and check if we can use waiting time
    auto [agents_to_use_waiting_time, can_use_waiting_time] = choose_agents_to_use_waiting_time(current_conflicts, current_solution);

    //initialize start and end time, old_end_t is the end time of the previous iteration
    const int zone_start_t = start_t; //zone start time never changes
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

        //build cnf 
        CNFConstructor cnf_constructor(local_zone_state.local_mdds, true);
        CNF local_cnf = cnf_constructor.construct_cnf();

        //add collision clauses to cnf  
        //WHAT ABOUT PSEUDO AGENTS? -> shoulc be ok since we keep the same ones until we have solution
        //when recreating the local zone we need to make sure the pseudo agents of an agent get the same pseudo agent id as before
        //then we can keep track of all collisions
        //otherwise we can only use the initial collisions
        cnf_constructor.add_collision_clauses_to_cnf(local_cnf, );
        cnf_constructor.add_edge_collision_clauses_to_cnf(local_cnf, );

        //Try lazy SAT solve the local zone
        auto lazy_sat_result = lazy_SAT_solve(
            local_zone_state.local_cnf,
            local_zone_state.cnf_constructor,
            local_zone_state.local_entry_exit_time,
            zone_start_t, new_end_t,
            10000, // max_iterations
            local_zone_state.initial_vertex_collisions,
            local_zone_state.initial_edge_collisions);

        if (lazy_sat_result.solution_found) {
            solution_found = true;
            std::cout << "[Waiting_time_Solve] Successfully solved local zone with waiting time" << std::endl;
            
        }

        std::cout << "[Waiting_time_Solve] Local zone state: " << local_zone_state.local_zone_paths.size() << " agents" << std::endl;
        std::cout << "[Waiting_time_Solve] number of pseudo agents: " << local_zone_state.original_agent_id.size() << std::endl;

        //update local_zone_paths
        local_zone_paths = local_zone_state.local_zone_paths;
        local_entry_exit_time = local_zone_state.local_entry_exit_time;
        local_mdds = local_zone_state.local_mdds;
        original_agent_id = local_zone_state.original_agent_id;


        //update current conflicts -> latest discovered collisions
        // handle pseudo agents conflicts

        //update which agents to use waiting time for the next iteration
        [agents_to_use_waiting_time, can_use_waiting_time] = choose_agents_to_use_waiting_time(current_conflicts, current_solution);
        //increase waiting time amount
        using_waiting_time += 1;
        // use that waiting time
        current_solution.use_waiting_time(agents_to_use_waiting_time, using_waiting_time);
        //increase exit time for the agents that use waiting time

        //if new exit time of an agent is greater than new_end_t, update new_end_t

        //

    //handle pseudo agents
    //if fail restore waiting time



    }
    return lazy_result;
}