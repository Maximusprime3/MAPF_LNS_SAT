#ifndef MDD_H
#define MDD_H

#include <unordered_map>
#include <vector>
#include <memory>
#include <random>
#include "MDDNode.h"

// MDD manages the Multi-value Decision Diagram (MDD) structure
// It stores nodes at each time step and provides methods to manipulate the MDD
class MDD {
public:
    // Maps time step to a list of nodes at that time
    std::unordered_map<int, std::vector<std::shared_ptr<MDDNode>>> levels;

    // Constructor: initializes an empty MDD
    MDD();
    // Adds a node to the appropriate time step level
    void add_node(const std::shared_ptr<MDDNode>& node);
    // Removes a node from the MDD and from its parents' children
    void remove_node(const std::shared_ptr<MDDNode>& node);
    // Prunes nodes with no children (except at the last time step)
    void prune_dead_ends(int max_timesteps);
    // Prunes nodes that are not reachable from the root (no parents except at root level)
    void prune_unreachable_nodes(int max_timesteps);
    // Prints each node and shows its children and parents
    void print_node_relationships() const;
    // Checks if any level from 0 to max_timesteps is empty
    bool is_broken(int max_timesteps) const;
    // Returns a deep copy of the MDD
    std::shared_ptr<MDD> copy() const;
    // Returns all nodes at a given level (time step)
    std::vector<std::shared_ptr<MDDNode>> get_nodes_at_level(int level) const;
    // Samples a random path from the root to a leaf using the MDD structure
    std::vector<MDDNode::Position> sample_random_path(std::mt19937& rng, std::shared_ptr<MDDNode> start = nullptr) const;
    // Returns a string representation of the MDD for printing
    std::string to_string() const;
};

#endif // MDD_H 