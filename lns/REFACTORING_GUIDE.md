# LNS Solver Refactoring Guide

## Overview

This document describes the refactoring of the original `LNSSolver.cpp` into a more modular, maintainable codebase using helper classes. The refactoring addresses the "spaghetti code" issues in the original implementation by breaking down complex functionality into focused, reusable components.

## Original Problems Identified

### 1. **Monolithic Main Function**
- The original `run_crude_lns()` function was over 1000 lines long
- Complex nested loops and conditional logic
- Difficult to understand, test, and maintain

### 2. **Repeated Code Patterns**
- MDD construction logic repeated multiple times
- Collision tracking scattered throughout the code
- Path manipulation code duplicated
- Zone expansion logic complex and hard to follow

### 3. **Poor Separation of Concerns**
- Conflict detection, bucket creation, and solving all mixed together
- No clear boundaries between different responsibilities
- Hard to modify one aspect without affecting others

## Refactoring Solution

### Helper Classes Created

#### 1. **CollisionTracker** (`LNSHelpers.h:15-45`)
**Purpose**: Centralized collision discovery and tracking
```cpp
class CollisionTracker {
    // Manages vertex and edge collisions
    // Provides merge, clear, and summary functionality
    // Replaces scattered collision tracking throughout original code
};
```

**Benefits**:
- Single source of truth for collision data
- Easy to merge collision information from different sources
- Consistent collision handling across the solver

#### 2. **ConflictBucketManager** (`LNSHelpers.h:50-85`)
**Purpose**: Handles conflict zone creation and management
```cpp
class ConflictBucketManager {
    // Creates diamond-shaped conflict buckets
    // Manages spatial conflict queries
    // Handles bucket statistics and validation
};
```

**Benefits**:
- Encapsulates complex bucket creation logic
- Provides efficient spatial conflict queries
- Makes bucket management reusable and testable

#### 3. **MDDManager** (`LNSHelpers.h:90-125`)
**Purpose**: Manages MDD construction and operations
```cpp
class MDDManager {
    // Creates MDDs for conflict zones
    // Handles MDD expansion for waiting time strategy
    // Manages MDD alignment and time window operations
};
```

**Benefits**:
- Centralizes MDD construction patterns
- Handles complex MDD operations like expansion and alignment
- Makes MDD management consistent and reusable

#### 4. **ZoneExpansionManager** (`LNSHelpers.h:130-165`)
**Purpose**: Handles conflict zone expansion logic
```cpp
class ZoneExpansionManager {
    // Manages progressive zone expansion
    // Handles conflict absorption during expansion
    // Provides zone coverage analysis
};
```

**Benefits**:
- Encapsulates complex expansion logic
- Handles iterative conflict discovery
- Provides zone coverage analysis for decision making

#### 5. **PathManager** (`LNSHelpers.h:170-205`)
**Purpose**: Handles path operations and solution updates
```cpp
class PathManager {
    // Extracts local paths from global solution
    // Updates global solution with local paths
    // Validates path consistency
};
```

**Benefits**:
- Centralizes path manipulation logic
- Provides consistent path validation
- Makes path operations reusable and safe

#### 6. **WaitingTimeStrategyManager** (`LNSHelpers.h:210-245`)
**Purpose**: Handles waiting time conflict resolution strategy
```cpp
class WaitingTimeStrategyManager {
    // Executes waiting time strategy
    // Manages MDD expansion for waiting time
    // Handles waiting time allocation
};
```

**Benefits**:
- Encapsulates complex waiting time logic
- Provides consistent waiting time strategy execution
- Makes waiting time management reusable

## Code Comparison

### Original Code Structure
```cpp
int run_crude_lns(...) {
    // 1000+ lines of mixed functionality:
    // - Problem loading
    // - MDD construction
    // - Path sampling
    // - Conflict detection
    // - Bucket creation
    // - Zone expansion
    // - SAT solving
    // - Solution updates
    // - All mixed together with complex control flow
}
```

### Refactored Code Structure
```cpp
int run_refactored_lns(...) {
    // Clean, focused main function:
    
    // 1. Initialize helper classes
    CollisionTracker global_collision_tracker;
    ConflictBucketManager bucket_manager(rows, cols, 2);
    MDDManager mdd_manager(problem.grid);
    // ... other managers
    
    // 2. Main solving loop
    for (int inc = 0; inc <= max_timestep_increase; ++inc) {
        // 3. Sample paths using PathManager
        auto sampled_paths = path_manager.sample_paths_from_mdds(mdds, rng);
        
        // 4. Create conflict buckets using ConflictBucketManager
        auto diamond_buckets = bucket_manager.create_diamond_buckets(...);
        
        // 5. Process each bucket
        for (const auto& bucket : diamond_buckets) {
            // 6. Try waiting time strategy using WaitingTimeStrategyManager
            auto waiting_time_result = waiting_time_manager.execute_strategy(...);
            
            // 7. If failed, try zone expansion using ZoneExpansionManager
            if (!waiting_time_result.solution_found) {
                auto expansion_result = expansion_manager.expand_zone(...);
            }
        }
    }
}
```

## Benefits of Refactoring

### 1. **Improved Readability**
- Each helper class has a single, clear responsibility
- Main function is now focused on high-level orchestration
- Complex logic is encapsulated and well-documented

### 2. **Better Maintainability**
- Changes to collision tracking only affect `CollisionTracker`
- MDD operations are centralized in `MDDManager`
- Zone expansion logic is isolated in `ZoneExpansionManager`

### 3. **Enhanced Testability**
- Each helper class can be unit tested independently
- Mock objects can be easily created for testing
- Complex interactions can be tested in isolation

### 4. **Increased Reusability**
- Helper classes can be reused in other MAPF solvers
- Common patterns are now available as reusable components
- New strategies can be implemented by extending existing classes

### 5. **Reduced Complexity**
- Main function is now ~200 lines instead of 1000+
- Each helper class is focused and manageable
- Complex nested logic is broken down into digestible pieces

## Usage Examples

### Using CollisionTracker
```cpp
CollisionTracker tracker;
tracker.add_vertex_collisions(vertex_collisions);
tracker.add_edge_collisions(edge_collisions);
tracker.print_summary("[LNS] ");
```

### Using ConflictBucketManager
```cpp
ConflictBucketManager bucket_manager(rows, cols, 2);
auto buckets = bucket_manager.create_diamond_buckets(
    conflict_points, conflict_meta, solved_conflict_indices);
bucket_manager.print_bucket_statistics(buckets);
```

### Using MDDManager
```cpp
MDDManager mdd_manager(grid);
auto local_mdds = mdd_manager.create_zone_mdds(
    local_zone_paths, local_entry_exit_time, masked_map, start_t, end_t);
```

### Using ZoneExpansionManager
```cpp
ZoneExpansionManager expansion_manager(rows, cols);
auto expansion_result = expansion_manager.expand_zone(
    original_conflicts, original_indices, expansion_factor, base_offset,
    all_conflict_points, conflict_map, solved_conflict_indices);
```

## Migration Guide

### For Existing Code
1. **Include the new header**: `#include "LNSHelpers.h"`
2. **Replace direct collision tracking** with `CollisionTracker` usage
3. **Replace bucket creation logic** with `ConflictBucketManager` calls
4. **Replace MDD construction** with `MDDManager` methods
5. **Replace zone expansion** with `ZoneExpansionManager` operations

### For New Features
1. **Use existing helper classes** for common operations
2. **Extend helper classes** for new functionality
3. **Create new helper classes** for new responsibilities
4. **Keep main function focused** on high-level orchestration

## Performance Considerations

### Memory Usage
- Helper classes add minimal memory overhead
- Object creation is amortized across multiple operations
- Memory management is handled automatically by smart pointers

### Computational Overhead
- Function call overhead is negligible compared to SAT solving
- Helper classes actually improve performance by reducing code duplication
- Better cache locality due to focused data structures

## Future Enhancements

### Potential Extensions
1. **Parallel Processing**: Helper classes can be easily parallelized
2. **Caching**: Add caching to frequently used operations
3. **Metrics**: Add performance metrics to helper classes
4. **Configuration**: Make helper classes configurable
5. **Plugins**: Allow custom strategies via plugin architecture

### Additional Helper Classes
1. **StatisticsManager**: Track solver performance metrics
2. **ConfigurationManager**: Handle solver configuration
3. **LoggingManager**: Centralized logging and debugging
4. **ValidationManager**: Solution validation and verification

## Conclusion

The refactoring successfully addresses the "spaghetti code" issues in the original LNS solver by:

1. **Breaking down complex functionality** into focused, manageable components
2. **Improving code organization** with clear separation of concerns
3. **Enhancing maintainability** through modular design
4. **Increasing reusability** of common patterns
5. **Reducing complexity** in the main solving logic

The new architecture makes the codebase more professional, maintainable, and extensible while preserving all the original functionality. The helper classes provide a solid foundation for future enhancements and can serve as a model for refactoring other complex algorithms in the codebase.
