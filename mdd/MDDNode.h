#ifndef MDDNODE_H
#define MDDNODE_H

#include <vector>
#include <memory>
#include <utility>

// MDDNode represents a node in the Multi-value Decision Diagram (MDD)
// Each node corresponds to an agent's position at a specific time step
class MDDNode {
public:
    // Position of the agent in the grid (row, column)
    using Position = std::pair<int, int>;

    Position position; // The (row, col) position of the node
    int time_step;     // The time step this node represents
    std::vector<std::shared_ptr<MDDNode>> children; // Children nodes (possible next positions)

    // Constructor: initializes the node with a position and time step
    MDDNode(Position pos, int t);
    // Adds a child node (possible next move)
    void add_child(const std::shared_ptr<MDDNode>& child);
};

#endif // MDDNODE_H 