# MDDConstructor Project

This project implements a Multi-value Decision Diagram (MDD) constructor in C++. It provides functionality to build MDDs for grid-based pathfinding problems, supporting features like obstacle handling, shortest path calculation, and flexible time-step management.

## Repository Overview

This repository contains code for constructing and manipulating Multi-value Decision Diagrams (MDDs) for grid-based pathfinding problems. The main components are organized as follows:

```
LNS/
├── mdd/
│   ├── MDDConstructor.cpp
│   ├── MDDConstructor.h
│   ├── MDD.cpp
│   ├── MDD.h
│   ├── MDDNode.cpp
│   ├── MDDNode.h
│   └── MDD.py
├── main.cpp         # Example usage and tests for the MDD classes
├── .gitignore       # Standard ignore rules for C++ projects
├── README.md        # Project documentation and build instructions
├── probSAT-master/  # probSAT SAT solver (CLI and C/C++ API)
```

- The `mdd/` folder contains all C++ (and Python) source and header files related to MDD construction and manipulation.
- `main.cpp` demonstrates usage and provides tests for the MDD classes.
- `.gitignore` and `README.md` are for project management and documentation.
- `probSAT-master/` contains the probSAT SAT solver, usable as a command-line tool or as a C/C++ library (see below).

## SAT Solving with probSAT

The repository includes the [probSAT SAT solver](probSAT-master/README.md), which can be used for solving SAT problems generated from MDDs or other sources.

### Usage Scenarios

- **Command-Line Interface (CLI):**
  - Build with `make` inside `probSAT-master/`.
  - Run with: `./probSAT instance.cnf <seed>`
  - See all options: `./probSAT -h`

- **C/C++ In-Memory API:**
  - Use the `probsat_solve_in_memory` function to solve SAT problems directly from C/C++ code, without file I/O.
  - Example usage:

```c
#include "probSAT_inmem.c"
// Example: Solve (x1 OR ~x2) AND (~x1 OR x2)
int main() {
    int numVars = 2;
    int numClauses = 2;
    int clause1[] = { 1, -2, 0 };   // x1 OR ~x2
    int clause2[] = { -1, 2, 0 };   // ~x1 OR x2
    int* clauses[3];                // 1-based indexing
    clauses[1] = clause1;
    clauses[2] = clause2;
    ProbSatResult result;
    int status = probsat_solve_in_memory(
        numVars,
        numClauses,
        clauses,
        42,         // seed
        10,         // maxRuns
        1000,       // maxFlips
        &result,
        NULL        // initialAssignment (NULL = random)
    );
    if (result.sat == 10) {
        printf("SAT found! Assignment: ");
        for (int i = 0; i < numVars; ++i) printf("%d ", result.assignment[i]);
        printf("\n");
    } else {
        printf("UNSAT or not found.\n");
    }
    free(result.assignment);
    return 0;
}
```
// Each line in the example is commented for clarity, as per user preference.

For full documentation, see [`probSAT-master/README.md`](probSAT-master/README.md).

## Building the Project

To build the project, use the following command:

```
g++ -std=c++17 -O2 -o main mdd/MDDConstructor.cpp mdd/MDD.cpp mdd/MDDNode.cpp
```

Replace `main` with your desired output executable name, and ensure all required source files are listed.

## Usage
- The code is designed for research and educational purposes related to pathfinding and decision diagrams.
- See the source files for more details on usage and implementation. 