# CNF Module

This module provides functionality to convert Multi-value Decision Diagrams (MDDs) to Conjunctive Normal Form (CNF) formulas for use with SAT solvers in Multi-Agent Path Finding (MAPF) problems.

## Overview

The CNF module consists of two main classes:

1. **CNF**: Basic CNF formula management with DIMACS format support
2. **CNFConstructor**: Converts MDDs to CNF formulas with various constraints

## Files

- `CNF.h` / `CNF.cpp`: Basic CNF formula class
- `CNFConstructor.h` / `CNFConstructor.cpp`: Main CNF construction logic
- `cnf_test.cpp`: Example usage and testing
- `Makefile`: Build configuration
- `README.md`: This documentation

## Features

### CNF Class
- Manage clauses and variables
- DIMACS format output
- Variable counting and clause management
- File I/O support

### CNFConstructor Class
- **Agent Path Constraints**: Ensures agents follow valid paths in their MDDs
- **Single Occupancy Constraints**: Each agent occupies only one position per timestep
- **Collision Avoidance**: Prevents agents from occupying the same position simultaneously
- **Edge Collision Prevention**: Prevents agents from swapping positions by traversing the same edge in opposite directions
- **Transition Constraints**: Ensures valid moves between consecutive timesteps
- **Lazy Encoding**: Option to defer conflict clauses for efficiency
- **Custom Collision Clauses**: Add specific collision constraints
- **Custom Edge Collision Clauses**: Add specific edge collision constraints
- **Path-CNF Translation**: Convert between agent paths and CNF variable assignments
- **Path Validation**: Verify paths are consistent with MDD constraints

## Usage

### Basic Usage

```cpp
#include "CNFConstructor.h"
#include "../mdd/MDDConstructor.h"

// Create MDDs for agents
std::unordered_map<int, std::shared_ptr<MDD>> mdds;
// ... populate mdds with agent MDDs ...

// Create CNF constructor
CNFConstructor cnf_constructor(mdds, false); // Non-lazy encoding

// Construct CNF
CNF cnf = cnf_constructor.construct_cnf();

// Output in DIMACS format
std::cout << cnf.to_dimacs();
```

### Lazy Encoding

```cpp
// Use lazy encoding for better performance with large problems
// Lazy encoding excludes collision and edge conflict clauses from initial construction
CNFConstructor lazy_constructor(mdds, true);
CNF lazy_cnf = lazy_constructor.construct_cnf();

// Eager encoding (default) includes all collision and edge conflict clauses
CNFConstructor eager_constructor(mdds, false);
CNF eager_cnf = eager_constructor.construct_cnf();
```

**Lazy vs Eager Encoding:**
- **Eager Encoding**: Includes all possible collision and edge conflict clauses upfront
- **Lazy Encoding**: Excludes collision and edge conflict clauses, allowing incremental addition of specific constraints
- **Use Case**: Lazy encoding is beneficial for large problems where most conflicts are unlikely to occur

### Adding Custom Collisions

```cpp
// Define specific collision constraints
std::vector<std::tuple<int, int, MDDNode::Position, int>> collisions = {
    {0, 1, {1, 0}, 2} // Agents 0 and 1 cannot both be at (1,0) at timestep 2
};

CNF cnf = cnf_constructor.construct_cnf(collisions);
```

### Edge Collision Clauses

```cpp
// Add edge collision clauses to prevent position swapping
CNF cnf_with_edge_collisions = cnf;
cnf_constructor.add_edge_collision_clauses_to_cnf(cnf_with_edge_collisions);

// Add a specific edge collision clause (for lazy encoding)
std::vector<int> edge_clause = cnf_constructor.add_single_edge_collision_clause_to_cnf(
    cnf, 0, 1, {1, 0}, {2, 0}, 2); // Agents 0 and 1 swapping at timestep 2

// Add a specific edge collision clause (UNCHECKED - no safety validation, faster)
std::vector<int> edge_clause_fast = cnf_constructor.add_single_edge_collision_clause_to_cnf_unchecked(
    cnf, 0, 1, {1, 0}, {2, 0}, 2); // Use when you're sure the positions exist in MDDs
```

### Performance Optimizations

For high-performance scenarios where you're confident about the validity of your constraints:

```cpp
// Unchecked collision clause (faster, no validation)
std::vector<int> collision_clause = cnf_constructor.add_single_collision_clause_unchecked(
    0, 1, {1, 0}, 2, false); // Returns clause without adding to CNF

// Unchecked edge collision clause (faster, no validation)
std::vector<int> edge_clause = cnf_constructor.add_single_edge_collision_clause_to_cnf_unchecked(
    cnf, 0, 1, {1, 0}, {2, 0}, 2); // Use when positions are guaranteed to exist
```

**Note:** Unchecked methods skip validation and will create new variables if they don't exist. Use only when you're certain about the validity of your constraints.

### Path-CNF Translation

```cpp
// Convert agent path to CNF variable assignments
std::vector<MDDNode::Position> path = {{0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2}};
std::vector<int> assignment = cnf_constructor.path_to_cnf_assignment(0, path);

// Convert CNF assignment back to agent paths
auto agent_paths = cnf_constructor.cnf_assignment_to_paths(assignment);

// Validate if a path is consistent with the MDD
bool is_valid = cnf_constructor.validate_path(0, path);

// Get variable ID for specific (agent, position, timestep)
int var_id = cnf_constructor.get_variable_id(0, {1, 0}, 2);
```

## Building

```bash
cd cnf/
make
```

## Testing

```bash
make test
```

This will run the test program and generate a `test.cnf` file in DIMACS format.

## DIMACS Format

The CNF is output in standard DIMACS format:

```
p cnf <variables> <clauses>
<clause1> 0
<clause2> 0
...
```

Where each clause is a space-separated list of literals ending with 0.

## Integration with SAT Solvers

The generated CNF can be used with any SAT solver that accepts DIMACS format:

```bash
# Example with MiniSAT
minisat test.cnf test.sat
```

## Logic Analysis

The implementation includes several key logical components:

1. **Path Validity**: Each agent must be at a valid node from their MDD at each timestep
2. **Mutual Exclusion**: No two agents can occupy the same position simultaneously
3. **Single Assignment**: Each agent can only be at one position per timestep
4. **Transition Consistency**: If an agent is at position p at time t, they must move to a valid child position at time t+1

## Performance Considerations

- **Lazy Encoding**: Use for large problems to reduce initial clause count
- **Variable Mapping**: Efficient lookup using tuples as keys
- **Memory Management**: Uses shared pointers for MDD nodes
- **Clause Optimization**: Avoids redundant clauses where possible

## Dependencies

- C++17 or later
- Standard library containers (unordered_map, vector, tuple)
- MDD module (../mdd/) 