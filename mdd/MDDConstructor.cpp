#include "MDDConstructor.h"
#include "MDD.h"
#include <set>
#include <algorithm>
#include <iostream>
#include <stdexcept>

// Constructor: initializes the grid, start, goal, and optionally max_timesteps and distances
MDDConstructor::MDDConstructor(const std::vector<std::vector<char>>& grid_, MDDNode::Position start_, MDDNode::Position goal_, int max_timesteps_, const std::unordered_map<MDDNode::Position, int, pair_hash>& distances_)
    : grid(grid_), start(start_), goal(goal_), max_timesteps(max_timesteps_), distances(distances_) {
    rows = grid.size();
    cols = grid.empty() ? 0 : grid[0].size();
    
    // Validate grid dimensions
    if (rows == 0) {
        throw std::invalid_argument("Grid cannot be empty");
    }
    
    // Validate start position
    if (start.first < 0 || start.first >= rows || start.second < 0 || start.second >= cols) {
        throw std::invalid_argument("Start position is out of bounds");
    }
    if (grid[start.first][start.second] != '.' && grid[start.first][start.second] != 'G') {
        throw std::invalid_argument("Start position is not on a free cell");
    }
    
    // Validate goal position
    if (goal.first < 0 || goal.first >= rows || goal.second < 0 || goal.second >= cols) {
        throw std::invalid_argument("Goal position is out of bounds");
    }
    if (grid[goal.first][goal.second] != '.' && grid[goal.first][goal.second] != 'G') {
        throw std::invalid_argument("Goal position is not on a free cell");
    }
    
    mdd = std::make_shared<MDD>();
}

// Returns valid neighbors (up, down, left, right) that are not obstacles
std::vector<MDDNode::Position> MDDConstructor::get_neighbors(const MDDNode::Position& pos) const {
    static const std::vector<std::pair<int, int>> directions = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1} // Up, Down, Left, Right
    };
    std::vector<MDDNode::Position> neighbors;
    int x = pos.first, y = pos.second;
    for (const auto& [dx, dy] : directions) {
        int nx = x + dx, ny = y + dy;
        // Check bounds and if cell is open or goal
        if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && (grid[nx][ny] == '.' || grid[nx][ny] == 'G')) {
            neighbors.emplace_back(nx, ny);
        }
    }
    return neighbors;
}

// Computes minimum distances from all positions to the goal using Dijkstra's algorithm
std::unordered_map<MDDNode::Position, int, pair_hash> MDDConstructor::compute_all_distances() {
    std::unordered_map<MDDNode::Position, int, pair_hash> dists;
    // Min-heap priority queue: (distance, position)
    std::priority_queue<std::pair<int, MDDNode::Position>, std::vector<std::pair<int, MDDNode::Position>>, std::greater<>> heap;
    heap.push({0, goal});
    while (!heap.empty()) {
        auto [distance, position] = heap.top();
        heap.pop();
        if (dists.count(position)) continue; // Already visited
        dists[position] = distance;
        for (const auto& neighbor : get_neighbors(position)) {
            if (!dists.count(neighbor)) {
                heap.push({distance + 1, neighbor});
            }
        }
    }
    return dists;
}

// Constructs the MDD and returns a shared pointer to it
std::shared_ptr<MDD> MDDConstructor::construct_mdd() {
    // If distances are not provided, compute them
    if (distances.empty()) {
        distances = compute_all_distances();
    }
    // If max_timesteps is not set, use the shortest path length from start to goal
    if (max_timesteps < 0) {
        auto it = distances.find(start);
        max_timesteps = (it != distances.end()) ? it->second : 0;
        // Print the value when it is set automatically
        std::cout << "[MDDConstructor] max_timesteps set to shortest path length from start to goal: " << max_timesteps << std::endl;
    }
    // Queue for BFS: (position, time_step)
    std::queue<std::pair<MDDNode::Position, int>> queue;
    std::set<std::pair<MDDNode::Position, int>> visited;
    // Create the start node at time 0
    auto start_node = std::make_shared<MDDNode>(start, 0);
    nodes[0][start] = start_node;
    mdd->add_node(start_node);
    queue.push({start, 0});
    // BFS to build the MDD
    while (!queue.empty()) {
        auto [position, current_time_step] = queue.front();
        queue.pop();
        if (visited.count({position, current_time_step})) continue;
        visited.insert({position, current_time_step});
        auto node = nodes[current_time_step][position];
        // Only expand if not at max_timesteps
        if (current_time_step < max_timesteps) {
            // For each neighbor and waiting in place
            auto next_positions = get_neighbors(position);
            next_positions.push_back(position); // Wait in place
            for (const auto& next_pos : next_positions) {
                int next_time_step = current_time_step + 1;
                // Only add if can still reach goal in time
                if (distances.count(next_pos) && distances[next_pos] + next_time_step <= max_timesteps) {
                    // Create node if it doesn't exist
                    if (!nodes[next_time_step].count(next_pos)) {
                        auto next_node = std::make_shared<MDDNode>(next_pos, next_time_step);
                        nodes[next_time_step][next_pos] = next_node;
                        mdd->add_node(next_node);
                        queue.push({next_pos, next_time_step});
                    }
                    // Add as child
                    node->add_child(nodes[next_time_step][next_pos]);
                }
            }
        }
    }
    return mdd;
} 