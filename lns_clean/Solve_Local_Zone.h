

struct LocalZoneResult {
    bool solution_found;
    std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths;
    std::vector<int> local_entry_exit_time;
};