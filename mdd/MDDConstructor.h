#ifndef MDDCONSTRUCTOR_H
#define MDDCONSTRUCTOR_H

#include <vector>
#include <unordered_map>
#include <memory>
#include <utility>
#include <queue>
#include <limits>
#include "MDDNode.h"

// Hash function for std::pair<int, int> to use in unordered_map
struct pair_hash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

class MDD; // Forward declaration

// MDDConstructor builds an MDD from a grid, start, and goal
class MDDConstructor {
public:
    // The grid: 2D vector of chars ('.' for open, 'G' for goal, etc.)
    std::vector<std::vector<char>> grid;
    // Start and goal positions
    MDDNode::Position start;
    MDDNode::Position goal;
    // Number of rows and columns in the grid
    int rows, cols;
    // Maximum allowed timesteps for the MDD
    int max_timesteps;
    // The constructed MDD
    std::shared_ptr<MDD> mdd;
    // Map from position to minimum distance to goal
    std::unordered_map<MDDNode::Position, int, pair_hash> distances;
    // Map from (time, position) to node
    std::unordered_map<int, std::unordered_map<MDDNode::Position, std::shared_ptr<MDDNode>, pair_hash>> nodes;

    // Constructor: initializes with grid, start, goal, and optionally max_timesteps and distances
    MDDConstructor(const std::vector<std::vector<char>>& grid_, MDDNode::Position start_, MDDNode::Position goal_, int max_timesteps = -1, const std::unordered_map<MDDNode::Position, int, pair_hash>& distances_ = {});

    // Returns valid neighbors (not obstacles) of a position
    std::vector<MDDNode::Position> get_neighbors(const MDDNode::Position& pos) const;

    // Computes minimum distances from all positions to the goal using Dijkstra's algorithm
    std::unordered_map<MDDNode::Position, int, pair_hash> compute_all_distances();

    // Constructs the MDD and returns a shared pointer to it
    std::shared_ptr<MDD> construct_mdd();
};

#endif // MDDCONSTRUCTOR_H 