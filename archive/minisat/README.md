# MiniSAT Integration for MAPF_LNS_SAT

This folder contains all MiniSAT-related files for the MAPF_LNS_SAT project.

## Folder Structure

```
minisat/
├── README.md                           # This file
├── MINISAT_INTEGRATION.md              # Detailed integration documentation
├── minisat-master/                     # MiniSAT source code repository
├── minisat-wrapper.h                   # Real MiniSAT wrapper header (not yet working)
├── minisat-wrapper.cpp                 # Real MiniSAT wrapper implementation
├── simple-minisat-wrapper.h            # Simplified wrapper header (alternative approach)
├── mock-minisat-wrapper.h              # Mock wrapper header (currently used)
├── mock-minisat-wrapper.cpp            # Mock wrapper implementation (currently used)
├── test_minisat_integration.cpp        # Integration test
├── example_minisat_lns_usage.cpp       # Example showing LNS usage
└── Makefile_test_minisat               # Build system for testing
```

## Quick Start

### Build and Test
```bash
cd minisat
make -f Makefile_test_minisat
./test_minisat_integration
```

### Run LNS Example
```bash
cd minisat
make -f Makefile_test_minisat
./example_minisat_lns_usage
```

## Implementation Status

### ✅ Working (Mock Implementation)
- `mock-minisat-wrapper.h/cpp` - Mock implementation that compiles and runs
- Integration with SATSolverManager
- Test suite and examples

### 🔄 In Progress (Real Implementation)
- `minisat-wrapper.h/cpp` - Real MiniSAT integration (compilation issues)
- `simple-minisat-wrapper.h` - Alternative approach

### 📚 Documentation
- `MINISAT_INTEGRATION.md` - Complete integration guide
- `README.md` - This overview file

## Usage

The MiniSAT integration is used through the `SATSolverManager`:

```cpp
#include "SATSolverManager.h"

// Solve with MiniSAT
auto result = SATSolverManager::solve_cnf_with_minisat(cnf_constructor);

if (result.satisfiable) {
    // Process solution...
}
```

## Next Steps

1. **Fix Real MiniSAT**: Resolve compilation issues with `minisat-master/`
2. **Replace Mock**: Switch from mock to real implementation
3. **Performance Testing**: Compare MiniSAT vs ProbSAT on MAPF instances

## Files Description

- **`minisat-master/`**: Official MiniSAT source code (from GitHub)
- **`mock-*`**: Working mock implementation for testing
- **`minisat-wrapper.*`**: Real integration attempt (needs fixes)
- **`simple-*`**: Alternative simplified approach
- **`test_*`**: Integration tests
- **`example_*`**: Usage examples
- **`Makefile_*`**: Build system
- **`MINISAT_INTEGRATION.md`**: Complete documentation

## Integration with Main Project

The MiniSAT integration is designed to be a drop-in replacement for ProbSAT in the main MAPF_LNS_SAT workflow. See `MINISAT_INTEGRATION.md` for detailed usage instructions. 