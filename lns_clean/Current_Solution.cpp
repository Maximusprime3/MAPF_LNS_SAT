#include "Current_Solution.h"

//used globally
//adds edges twice for incex consistency, bc they have two conflictpoints
std::pair<std::vector<std::pair<int,int>>, std::vector<ConflictMeta>> collect_conflicts(
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& vertex_collisions,
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& edge_collisions) {
    std::vector<std::pair<int,int>> conflict_points;
    std::vector<ConflictMeta> conflict_meta ;
    conflict_points.reserve(vertex_collisions.size() + edge_collisions.size() * 2);
    conflict_meta.reserve(vertex_collisions.size() + edge_collisions.size() * 2);

    // Vertex conflicts contribute a single point and metadata entry.
    for (const auto& v : vertex_collisions) {
        int a1 = std::get<0>(v);
        int a2 = std::get<1>(v);
        auto pos = std::get<2>(v);
        int t = std::get<3>(v);
        conflict_points.push_back(pos);
        conflict_meta.push_back(ConflictMeta{a1, a2, t, false, pos, {-1, -1}});
    }

    // Edge conflicts contribute both end points to improve spatial clustering.
    for (const auto& e : edge_collisions) {
        int a1 = std::get<0>(e);
        int a2 = std::get<1>(e);
        auto pos1 = std::get<2>(e);
        auto pos2 = std::get<3>(e);
        int t = std::get<4>(e);
        conflict_points.push_back(pos1);
        conflict_meta.push_back(ConflictMeta{a1, a2, t, true, pos1, pos2});
        conflict_points.push_back(pos2);
        conflict_meta.push_back(ConflictMeta{a1, a2, t, true, pos1, pos2});
    }
    return [conflict_points, conflict_meta];
}

//used locallywithout conflict points
std::vector<ConflictMeta> collect_conflicts_meta(
    const std::vector<std::tuple<int, int, std::pair<int,int>, int>>& vertex_collisions,
    const std::vector<std::tuple<int, int, std::pair<int,int>, std::pair<int,int>, int>>& edge_collisions) {
    std::vector<ConflictMeta> conflict_meta;
    conflict_meta.reserve(vertex_collisions.size() + edge_collisions.size());

    // Vertex conflicts contribute a single metadata entry.
    for (const auto& v : vertex_collisions) {
        int a1 = std::get<0>(v);
        int a2 = std::get<1>(v);
        auto pos = std::get<2>(v);
        int t = std::get<3>(v);
        conflict_meta.push_back(ConflictMeta{a1, a2, t, false, pos, {-1, -1}});
    }

    // Edge conflicts contribute a single metadata entry.
    for (const auto& e : edge_collisions) {
        int a1 = std::get<0>(e);
        int a2 = std::get<1>(e);
        auto pos1 = std::get<2>(e);
        auto pos2 = std::get<3>(e);
        int t = std::get<4>(e);
        conflict_meta.push_back(ConflictMeta{a1, a2, t, true, pos1, pos2});
    }
    return conflict_meta;
}


// Helper: create a spatial conflict map for efficient conflict queries
// Returns a 2D array where conflict_map[r][c] = conflict_index if there's a conflict at (r,c), -1 otherwise
std::vector<std::vector<std::vector<int>>> create_conflict_map_2D(
    const std::vector<std::pair<int,int>>& conflict_points,
    int rows, int cols) {
    std::vector<std::vector<std::vector<int>>> conflict_map(rows, std::vector<std::vector<int>>(cols));
    
    for (size_t i = 0; i < conflict_points.size(); ++i) {
        auto [r, c] = conflict_points[i];
        if (r >= 0 && r < rows && c >= 0 && c < cols) {
            conflict_map[r][c].push_back(static_cast<int>(i));
        }
    }
    
    return conflict_map;
}