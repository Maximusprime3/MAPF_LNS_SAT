#include <iostream>
#include <vector>
#include <memory>
#include <random>
#include <algorithm>
#include "MDDConstructor.h"
#include "MDD.h"
#include "MDDNode.h"

int main() {
    // Seed for random number generation
    std::random_device rd;
    std::mt19937 rng(rd());

    // 1. Create a small 3x3 grid
    // '.' = open cell, 'G' = goal
    std::vector<std::vector<char>> grid = {
        {'.', '.', '.'},
        {'.', '.', '.'},
        {'.', '.', 'G'}
    };
    // Start at (0,0), goal at (2,2)
    MDDNode::Position start = {0, 0};
    MDDNode::Position goal = {2, 2};

    // 2. Construct the MDD
    int max_timesteps = 4; // or any value >= shortest path length
    MDDConstructor constructor(grid, start, goal, max_timesteps);
    std::shared_ptr<MDD> mdd = constructor.construct_mdd();

    // Debug: Print max_timesteps used
    std::cout << "max_timesteps used: " << constructor.max_timesteps << std::endl;

    // Debug: Print all nodes at each level
    std::cout << "Nodes at each level after construction:" << std::endl;
    for (int t = 0; t <= constructor.max_timesteps; ++t) {
        auto nodes = mdd->get_nodes_at_level(t);
        std::cout << "Level " << t << " (" << nodes.size() << " nodes): ";
        for (const auto& node : nodes) {
            std::cout << "(" << node->position.first << "," << node->position.second << ") ";
        }
        std::cout << std::endl;
    }

    // 3. Print the initial MDD and node relationships
    std::cout << "\nInitial MDD:\n" << mdd->to_string() << std::endl;
    mdd->print_node_relationships();

    // 4. Test 1: Add a node without children (should be pruned)
    std::cout << "\n=== Test 1: Adding node without children ===" << std::endl;
    auto isolated_node = std::make_shared<MDDNode>(MDDNode::Position{1, 1}, 2);
    mdd->add_node(isolated_node);
    std::cout << "MDD after adding isolated node at level 2 (1,1):\n" << mdd->to_string() << std::endl;
    mdd->print_node_relationships();
    
    // Prune dead ends - this should remove the isolated node
    mdd->prune_dead_ends(constructor.max_timesteps);
    std::cout << "MDD after pruning dead ends (isolated node should be removed):\n" << mdd->to_string() << std::endl;
    mdd->print_node_relationships();
    
    // Check if MDD is broken
    if (mdd->is_broken(constructor.max_timesteps)) {
        std::cout << "MDD is broken (some level is empty)!\n";
    } else {
        std::cout << "MDD is still valid (all levels have nodes).\n";
    }

    // 5. Test 2: Delete a node from level 1 (if it exists)
    std::cout << "\n=== Test 2: Deleting a node ===" << std::endl;
    auto level1_nodes = mdd->get_nodes_at_level(1);
    if (!level1_nodes.empty()) {
        std::cout << "Deleting node at level 1: (" << level1_nodes[0]->position.first << "," << level1_nodes[0]->position.second << ")\n";
        mdd->remove_node(level1_nodes[0]);
        std::cout << "MDD after deleting node:\n" << mdd->to_string() << std::endl;
        mdd->print_node_relationships();
        
        // Prune dead ends after deletion (removes children with no parents)
        mdd->prune_dead_ends(constructor.max_timesteps);
        std::cout << "MDD after pruning dead ends:\n" << mdd->to_string() << std::endl;
        mdd->print_node_relationships();
        
        // Prune unreachable nodes (removes nodes with no parents)
        mdd->prune_unreachable_nodes(constructor.max_timesteps);
        std::cout << "MDD after pruning unreachable nodes:\n" << mdd->to_string() << std::endl;
        mdd->print_node_relationships();
        
        // Check if the MDD is broken
        if (mdd->is_broken(constructor.max_timesteps)) {
            std::cout << "MDD is broken (some level is empty)!\n";
        } else {
            std::cout << "MDD is still valid (all levels have nodes).\n";
        }
    }

    // 6. Test 3: Add a node with proper connections
    std::cout << "\n=== Test 3: Adding node with proper connections ===" << std::endl;
    auto connected_node = std::make_shared<MDDNode>(MDDNode::Position{0, 1}, 1);
    
    // Connect to parent at level 0
    auto level0_nodes = mdd->get_nodes_at_level(0);
    if (!level0_nodes.empty()) {
        // Find the parent node at (0,0)
        for (const auto& parent : level0_nodes) {
            if (parent->position == MDDNode::Position{0, 0}) {
                parent->add_child(connected_node);
                std::cout << "Connected parent (" << parent->position.first << "," << parent->position.second 
                          << ") to (" << connected_node->position.first << "," << connected_node->position.second << ")\n";
                break;
            }
        }
    }
    
    // Find potential children at level 2 (next timestep)
    auto level2_nodes = mdd->get_nodes_at_level(2);
    for (const auto& child : level2_nodes) {
        // Check if this child is reachable from our new node
        // For simplicity, let's connect to all level 2 nodes that are neighbors
        auto neighbors = constructor.get_neighbors(connected_node->position);
        bool is_neighbor = false;
        for (const auto& neighbor : neighbors) {
            if (neighbor.first == child->position.first && neighbor.second == child->position.second) {
                is_neighbor = true;
                break;
            }
        }
        if (is_neighbor) {
            connected_node->add_child(child);
            std::cout << "Connected (" << connected_node->position.first << "," << connected_node->position.second 
                      << ") to (" << child->position.first << "," << child->position.second << ")\n";
        }
    }
    
    mdd->add_node(connected_node);
    std::cout << "MDD after adding connected node:\n" << mdd->to_string() << std::endl;
    mdd->print_node_relationships();
    
    // Prune dead ends
    mdd->prune_dead_ends(constructor.max_timesteps);
    std::cout << "MDD after pruning dead ends:\n" << mdd->to_string() << std::endl;
    mdd->print_node_relationships();
    
    // Prune unreachable nodes
    mdd->prune_unreachable_nodes(constructor.max_timesteps);
    std::cout << "MDD after pruning unreachable nodes:\n" << mdd->to_string() << std::endl;
    mdd->print_node_relationships();
    
    // Check if MDD is broken
    if (mdd->is_broken(constructor.max_timesteps)) {
        std::cout << "MDD is broken (some level is empty)!\n";
    } else {
        std::cout << "MDD is still valid (all levels have nodes).\n";
    }

    // 7. Sample a random path from the final MDD
    std::cout << "\n=== Final MDD State ===" << std::endl;
    auto path = mdd->sample_random_path(rng);
    std::cout << "Sampled random path: ";
    for (const auto& pos : path) {
        std::cout << "(" << pos.first << "," << pos.second << ") ";
    }
    std::cout << std::endl;

    return 0;
} 