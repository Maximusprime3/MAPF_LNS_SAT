#pragma once

#include <unordered_map>
#include <vector>

struct LocalZoneResult {
    bool solution_found;
    bool solution_found = false;
    std::unordered_map<int, std::vector<std::pair<int,int>>> local_paths;
    std::vector<int> local_entry_exit_time;
    std::unordered_map<int, std::pair<int,int>> local_entry_exit_time;
};