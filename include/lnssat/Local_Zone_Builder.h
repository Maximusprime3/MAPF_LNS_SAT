#pragma once

#include <set>
#include <utility>
#include <vector>

// Builds reachability-aware conflict zones that respect map geometry.
class ConflictZoneBuilder {
public:
    explicit ConflictZoneBuilder(const std::vector<std::vector<char>>& map);

    std::set<std::pair<int,int>> build_reachable_zone(
        const std::vector<std::pair<int,int>>& conflict_points,
        int expansion_radius) const;

private:
    bool in_bounds(int r, int c) const;
    bool is_walkable(int r, int c) const;
    int degree(int r, int c) const;
    void extend_corridor_chain(std::set<std::pair<int,int>>& seeds,
                               int start_r,
                               int start_c,
                               int from_r,
                               int from_c) const;

private:
    const std::vector<std::vector<char>>& map_;
    int rows_;
    int cols_;
};