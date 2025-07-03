#include "MDD.h"
#include "MDDNode.h"
#include <algorithm>
#include <sstream>
#include <unordered_set>
#include <iostream>

// Constructor: initializes an empty MDD
MDD::MDD() {}

// Adds a node to the appropriate time step level
void MDD::add_node(const std::shared_ptr<MDDNode>& node) {
    levels[node->time_step].push_back(node);
}

// Removes a node from the MDD and from its parents' children
void MDD::remove_node(const std::shared_ptr<MDDNode>& node) {
    int time_level = node->time_step;
    // Remove from the current level
    auto& nodes = levels[time_level];
    nodes.erase(std::remove(nodes.begin(), nodes.end(), node), nodes.end());
    // Remove from parents' children if not at the root
    if (time_level > 0) {
        auto& parent_list = levels[time_level - 1];
        for (auto& pnode : parent_list) {
            auto& children = pnode->children;
            children.erase(std::remove(children.begin(), children.end(), node), children.end());
        }
    }
}

// Prunes nodes with no children (except at the last time step)
void MDD::prune_dead_ends(int max_timesteps) {
    bool changed = true;
    // Repeat until no more nodes are removed in a pass
    while (changed) {
        changed = false;
        // For each layer from 0 to max_timesteps-1
        for (int t = 0; t < max_timesteps; ++t) {
            if (levels.find(t) == levels.end()) continue;
            std::vector<std::shared_ptr<MDDNode>> to_remove;
            // Find nodes with no children
            for (const auto& node : levels[t]) {
                if (node->children.empty()) {
                    to_remove.push_back(node);
                }
            }
            // Remove those nodes
            if (!to_remove.empty()) {
                for (const auto& node : to_remove) {
                    remove_node(node);
                }
                changed = true;
            }
        }
    }
}

// Prunes nodes that are not reachable from the root (no parents except at root level)
void MDD::prune_unreachable_nodes(int max_timesteps) {
    bool changed = true;
    // Repeat until no more nodes are removed in a pass
    while (changed) {
        changed = false;
        // For each layer from 1 to max_timesteps (skip root level 0)
        for (int t = 1; t <= max_timesteps; ++t) {
            if (levels.find(t) == levels.end()) continue;
            std::vector<std::shared_ptr<MDDNode>> to_remove;
            // Check each node at this level
            for (const auto& node : levels[t]) {
                bool has_parent = false;
                // Check if any parent at the previous level has this node as a child
                if (levels.find(t - 1) != levels.end()) {
                    for (const auto& parent : levels[t - 1]) {
                        for (const auto& child : parent->children) {
                            if (child == node) {
                                has_parent = true;
                                break;
                            }
                        }
                        if (has_parent) break;
                    }
                }
                // If no parent found, mark for removal
                if (!has_parent) {
                    to_remove.push_back(node);
                }
            }
            // Remove unreachable nodes
            if (!to_remove.empty()) {
                for (const auto& node : to_remove) {
                    remove_node(node);
                }
                changed = true;
            }
        }
    }
}

// Prints each node and shows its children and parents
void MDD::print_node_relationships() const {
    std::cout << "\n=== Node Relationships ===" << std::endl;
    for (const auto& [level, nodes] : levels) {
        std::cout << "Level " << level << ":" << std::endl;
        for (const auto& node : nodes) {
            std::cout << "  Node (" << node->position.first << "," << node->position.second 
                      << ") at time " << node->time_step << ":" << std::endl;
            
            // Print children
            std::cout << "    Children: ";
            if (node->children.empty()) {
                std::cout << "none";
            } else {
                for (size_t i = 0; i < node->children.size(); ++i) {
                    const auto& child = node->children[i];
                    std::cout << "(" << child->position.first << "," << child->position.second << ")";
                    if (i + 1 < node->children.size()) std::cout << ", ";
                }
            }
            std::cout << std::endl;
            
            // Print parents
            std::cout << "    Parents: ";
            if (level == 0) {
                std::cout << "root (no parents)";
            } else {
                std::vector<std::string> parent_strings;
                if (levels.find(level - 1) != levels.end()) {
                    for (const auto& potential_parent : levels.at(level - 1)) {
                        for (const auto& child : potential_parent->children) {
                            if (child == node) {
                                parent_strings.push_back("(" + std::to_string(potential_parent->position.first) + 
                                                       "," + std::to_string(potential_parent->position.second) + ")");
                                break;
                            }
                        }
                    }
                }
                if (parent_strings.empty()) {
                    std::cout << "none (unreachable!)";
                } else {
                    for (size_t i = 0; i < parent_strings.size(); ++i) {
                        std::cout << parent_strings[i];
                        if (i + 1 < parent_strings.size()) std::cout << ", ";
                    }
                }
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}

// Checks if any level from 0 to max_timesteps is empty
bool MDD::is_broken(int max_timesteps) const {
    for (int t = 0; t <= max_timesteps; ++t) {
        auto it = levels.find(t);
        if (it == levels.end() || it->second.empty()) {
            return true;
        }
    }
    return false;
}

// Returns a deep copy of the MDD
std::shared_ptr<MDD> MDD::copy() const {
    auto new_mdd = std::make_shared<MDD>();
    // Map from old node pointers to new node pointers
    std::unordered_map<const MDDNode*, std::shared_ptr<MDDNode>> old_to_new;
    // (A) Create all new nodes, same position/time_step, but empty children
    for (const auto& [t, node_list] : levels) {
        for (const auto& old_node : node_list) {
            auto new_node = std::make_shared<MDDNode>(old_node->position, old_node->time_step);
            old_to_new[old_node.get()] = new_node;
            new_mdd->add_node(new_node);
        }
    }
    // (B) Rebuild children references
    for (const auto& [t, node_list] : levels) {
        for (const auto& old_node : node_list) {
            auto new_node = old_to_new[old_node.get()];
            for (const auto& old_child : old_node->children) {
                new_node->children.push_back(old_to_new[old_child.get()]);
            }
        }
    }
    return new_mdd;
}

// Returns all nodes at a given level (time step)
std::vector<std::shared_ptr<MDDNode>> MDD::get_nodes_at_level(int level) const {
    auto it = levels.find(level);
    if (it != levels.end()) {
        return it->second;
    }
    return {};
}

// Samples a random path from the root to a leaf using the MDD structure
std::vector<MDDNode::Position> MDD::sample_random_path(std::mt19937& rng, std::shared_ptr<MDDNode> start) const {
    std::vector<MDDNode::Position> path;
    // If no start node is provided, use the first node at level 0
    if (!start) {
        auto it = levels.find(0);
        if (it == levels.end() || it->second.empty()) return path;
        start = it->second[0];
    }
    auto current_node = start;
    // Traverse the MDD by randomly selecting a child at each step
    while (current_node) {
        path.push_back(current_node->position);
        if (current_node->children.empty()) break;
        std::uniform_int_distribution<size_t> dist(0, current_node->children.size() - 1);
        current_node = current_node->children[dist(rng)];
    }
    return path;
}

// Returns a string representation of the MDD for printing
std::string MDD::to_string() const {
    std::ostringstream oss;
    for (const auto& [level, nodes] : levels) {
        oss << "Time Step " << level << ": ";
        for (size_t i = 0; i < nodes.size(); ++i) {
            const auto& node = nodes[i];
            oss << "(" << node->position.first << "," << node->position.second << ")";
            if (i + 1 < nodes.size()) oss << ", ";
        }
        oss << "\n";
    }
    return oss.str();
}
