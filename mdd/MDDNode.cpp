#include "MDDNode.h"

// Constructor: initializes the node with a given position and time step
MDDNode::MDDNode(Position pos, int t)
    : position(pos), time_step(t) {}

// Adds a child node to this node's list of children
void MDDNode::add_child(const std::shared_ptr<MDDNode>& child) {
    children.push_back(child);
} 