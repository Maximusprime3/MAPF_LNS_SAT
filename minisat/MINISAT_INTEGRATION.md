# MiniSAT Integration for MAPF_LNS_SAT

This document describes the integration of MiniSAT into the MAPF_LNS_SAT project, following the same pattern as the existing ProbSAT integration.

## Overview

MiniSAT is a minimalistic and high-performance SAT solver that uses a CDCL (Conflict-Driven Clause Learning) approach, which is different from ProbSAT's stochastic local search approach. This integration provides an alternative solver option for the MAPF (Multi-Agent Pathfinding) problem.

## Integration Architecture

### 1. MiniSAT Wrapper (`mock-minisat-wrapper.h/cpp`)

The integration uses a wrapper pattern similar to ProbSAT:

```cpp
class MockMiniSatWrapper {
public:
    // Main solving function
    MiniSatSolution solve_cnf(const std::vector<std::vector<int>>& clauses, 
                             const std::vector<int>* initial_assignment = nullptr);
    
    // Alternative interface for pointer-based CNF format
    MiniSatSolution solve_cnf_with_pointers(int numVars, 
                                           int numClauses, 
                                           int** clauses,
                                           const std::vector<int>* initial_assignment = nullptr);
private:
    // Internal solver state and helper methods
};
```

### 2. Result Structure

The `MiniSatSolution` struct matches the ProbSAT interface:

```cpp
struct MiniSatSolution {
    bool satisfiable;                    // Whether the problem is satisfiable
    std::vector<int> assignment;        // Variable assignments (1-based indexing)
    int num_decisions;                  // Number of decisions made (equivalent to flips in ProbSAT)
    double solve_time;                  // Time taken to solve (seconds)
    std::string error_message;          // Error message if solving failed
};
```

### 3. SATSolverManager Integration

Two new methods were added to `SATSolverManager`:

```cpp
// Solve CNF directly
static MiniSatSolution solve_cnf_with_minisat(const CNF& cnf,
                                             const std::vector<int>* initial_assignment = nullptr);

// Solve via CNFProbSATConstructor
static MiniSatSolution solve_cnf_with_minisat(const std::shared_ptr<CNFProbSATConstructor>& cnf_constructor,
                                             const std::vector<int>* initial_assignment = nullptr);
```

## Usage Example

```cpp
#include "SATSolverManager.h"

// Create CNF from MDDs
auto cnf_constructor = SATSolverManager::create_cnf_probsat_constructor(mdds, true);

// Solve with MiniSAT
auto minisat_result = SATSolverManager::solve_cnf_with_minisat(cnf_constructor);

if (minisat_result.satisfiable) {
    std::cout << "MiniSAT: SATISFIABLE!" << std::endl;
    std::cout << "Solve time: " << minisat_result.solve_time << "s" << std::endl;
    std::cout << "Number of decisions: " << minisat_result.num_decisions << std::endl;
    
    // Extract agent paths
    auto agent_paths = cnf_constructor->cnf_assignment_to_paths(minisat_result.assignment);
    
    // Process results...
} else {
    std::cout << "MiniSAT: UNSATISFIABLE" << std::endl;
}
```

## Comparison with ProbSAT

| Aspect | ProbSAT | MiniSAT |
|--------|---------|---------|
| **Algorithm** | Stochastic Local Search | CDCL (Conflict-Driven Clause Learning) |
| **Strengths** | Good for satisfiable instances, fast for small problems | Complete solver, good for unsatisfiable instances |
| **Weaknesses** | Incomplete, may not find solution even if one exists | Can be slower for satisfiable instances |
| **Parameters** | `max_runs`, `max_flips`, `seed` | Fewer parameters, more deterministic |
| **Performance** | Very fast for satisfiable instances | More consistent performance |

## Current Implementation Status

### ✅ Completed
- Integration architecture and interfaces
- Mock implementation for testing
- SATSolverManager integration
- Test suite demonstrating functionality
- Build system integration

### 🔄 In Progress / TODO
- **Real MiniSAT Integration**: The current implementation uses a mock wrapper. To use the real MiniSAT:
  1. Fix compilation issues with MiniSAT source code
  2. Replace `MockMiniSatWrapper` with real `MiniSatWrapper`
  3. Update build system to compile MiniSAT source files

### 🚧 Known Issues
- MiniSAT source code has C++ compatibility issues (friend declarations with default arguments)
- Requires zlib development libraries
- Some compiler warnings about literal suffixes

## Building and Testing

### Prerequisites
```bash
# Install zlib development libraries (for real MiniSAT)
sudo apt-get install zlib1g-dev
```

### Build Test
```bash
# Build the MiniSAT integration test
make -f Makefile_test_minisat

# Run the test
./test_minisat_integration
```

### Integration with Main Project
To integrate MiniSAT into the main MAPF_LNS_SAT workflow:

1. **Replace mock with real implementation**:
   ```cpp
   // In SATSolverManager.h
   #include "minisat-wrapper.h"  // Instead of mock-minisat-wrapper.h
   ```

2. **Update build system**:
   ```makefile
   # Add MiniSAT source files
   MINISAT_SOURCES = minisat-master/minisat/core/Solver.cc \
                     minisat-master/minisat/utils/Options.cc \
                     minisat-master/minisat/utils/System.cc
   ```

3. **Use in LNS loop**:
   ```cpp
   // In the main LNS loop, replace ProbSAT with MiniSAT
   auto result = SATSolverManager::solve_cnf_with_minisat(cnf_constructor);
   ```

## Performance Considerations

### When to Use MiniSAT vs ProbSAT

**Use MiniSAT when:**
- You need a complete solver (guaranteed to find solution if one exists)
- Dealing with potentially unsatisfiable instances
- Want more deterministic behavior
- Need to prove unsatisfiability

**Use ProbSAT when:**
- You know the problem is likely satisfiable
- Need very fast solving for satisfiable instances
- Want to explore multiple solutions quickly
- Working with large instances where completeness is less important

### Hybrid Approach
Consider using both solvers in a hybrid approach:
1. Start with ProbSAT for fast initial attempts
2. If ProbSAT fails, switch to MiniSAT for completeness
3. Use MiniSAT to verify if the problem is truly unsatisfiable

## Future Enhancements

1. **Real MiniSAT Integration**: Complete the integration with actual MiniSAT source code
2. **Performance Benchmarking**: Compare MiniSAT vs ProbSAT on MAPF instances
3. **Hybrid Solver**: Implement automatic solver selection based on problem characteristics
4. **Parameter Tuning**: Optimize MiniSAT parameters for MAPF problems
5. **Parallel Solving**: Use multiple solver instances in parallel

## References

- [MiniSAT GitHub Repository](https://github.com/niklasso/minisat)
- [MiniSAT Paper](http://minisat.se/downloads/MiniSat.pdf)
- [ProbSAT Integration Documentation](probSAT-master/README.md)

## Conclusion

The MiniSAT integration provides a complete alternative to ProbSAT for solving MAPF problems encoded as SAT instances. While the current implementation uses a mock wrapper for demonstration purposes, the architecture is designed to easily accommodate the real MiniSAT solver once compilation issues are resolved.

The integration follows the same patterns as the existing ProbSAT integration, making it easy to switch between solvers or use them in combination for optimal performance. 