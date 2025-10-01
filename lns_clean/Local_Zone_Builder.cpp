#include "Local_Zone_Builder.h"

#include <array>
#include <queue>

namespace {
constexpr char kWalkable = '.';
constexpr char kGoal = 'G';

constexpr std::array<std::pair<int,int>, 4> kDirections{{
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}
}};
}

ConflictZoneBuilder::ConflictZoneBuilder(const std::vector<std::vector<char>>& map)
    : map_(map),
      rows_(static_cast<int>(map.size())),
      cols_(rows_ > 0 ? static_cast<int>(map[0].size()) : 0) {}

bool ConflictZoneBuilder::in_bounds(int r, int c) const {
    return r >= 0 && r < rows_ && c >= 0 && c < cols_;
}

bool ConflictZoneBuilder::is_walkable(int r, int c) const {
    if (!in_bounds(r, c)) {
        return false;
    }
    char cell = map_[r][c];
    return cell == kWalkable || cell == kGoal;
}

int ConflictZoneBuilder::degree(int r, int c) const {
    if (!is_walkable(r, c)) {
        return 0;
    }

    int count = 0;
    for (const auto& [dr, dc] : kDirections) {
        int nr = r + dr;
        int nc = c + dc;
        if (is_walkable(nr, nc)) {
            ++count;
        }
    }
    return count;
}

void ConflictZoneBuilder::extend_corridor_chain(std::set<std::pair<int,int>>& seeds,
                                                int start_r,
                                                int start_c,
                                                int from_r,
                                                int from_c) const {
    std::pair<int,int> prev{from_r, from_c};
    std::pair<int,int> cur{start_r, start_c};

    // Track nodes we've already added while extending this branch to avoid loops.
    std::set<std::pair<int,int>> local_seen;

    while (true) {
        if (!is_walkable(cur.first, cur.second)) {
            break;
        }

        if (!local_seen.insert(cur).second) {
            break;
        }

        seeds.insert(cur);

        if (degree(cur.first, cur.second) != 2) {
            break;
        }

        bool found_next = false;
        std::pair<int,int> next{};
        for (const auto& [dr, dc] : kDirections) {
            int nr = cur.first + dr;
            int nc = cur.second + dc;
            if (!is_walkable(nr, nc)) {
                continue;
            }
            if (nr == prev.first && nc == prev.second) {
                continue;
            }
            next = {nr, nc};
            found_next = true;
            break;
        }

        if (!found_next) {
            break;
        }

        prev = cur;
        cur = next;
    }
}

std::set<std::pair<int,int>> ConflictZoneBuilder::build_reachable_zone(
    const std::vector<std::pair<int,int>>& conflict_points,
    int expansion_radius) const {
    std::set<std::pair<int,int>> seeds;

    for (const auto& conflict_point : conflict_points) {
        int center_row = conflict_point.first;
        int center_col = conflict_point.second;
        if (!in_bounds(center_row, center_col)) {
            continue;
        }

        seeds.insert(conflict_point);

        if (degree(center_row, center_col) != 2) {
            continue;
        }

        for (const auto& [dr, dc] : kDirections) {
            int nr = center_row + dr;
            int nc = center_col + dc;
            if (!is_walkable(nr, nc)) {
                continue;
            }
            extend_corridor_chain(seeds, nr, nc, center_row, center_col);
        }
    }

    std::set<std::pair<int,int>> zone;
    if (seeds.empty() || expansion_radius < 0) {
        return zone;
    }

    std::vector<std::vector<int>> distance(rows_, std::vector<int>(cols_, -1));
    std::queue<std::pair<int,int>> frontier;

    for (const auto& seed : seeds) {
        int sr = seed.first;
        int sc = seed.second;
        if (!in_bounds(sr, sc)) {
            continue;
        }
        if (distance[sr][sc] != -1) {
            continue;
        }
        distance[sr][sc] = 0;
        frontier.push(seed);
    }

    while (!frontier.empty()) {
        auto [r, c] = frontier.front();
        frontier.pop();

        int dist = distance[r][c];
        if (dist > expansion_radius) {
            continue;
        }

        if (dist == 0 || is_walkable(r, c)) {
            zone.insert({r, c});
        } else {
            continue;
        }

        if (dist == expansion_radius) {
            continue;
        }

        for (const auto& [dr, dc] : kDirections) {
            int nr = r + dr;
            int nc = c + dc;
            if (!in_bounds(nr, nc) || !is_walkable(nr, nc)) {
                continue;
            }
            if (distance[nr][nc] != -1 && distance[nr][nc] <= dist + 1) {
                continue;
            }
            distance[nr][nc] = dist + 1;
            frontier.push({nr, nc});
        }
    }

    return zone;
}