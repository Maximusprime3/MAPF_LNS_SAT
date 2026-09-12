probSAT
=======

The probSAT SAT Solver

An efficient implementation of a variant of the probSAT solver presented in:

"Choosing Probability Distributions for Stochastic Local Search and the Role of Make versus Break" by
Adrian Balint, Uwe Schöning

published in Lecture Notes in Computer Science, 2012, Volume 7317, Theory and Applications of Satisfiability Testing - SAT 2012, pages 16-29

This version does not track the make values of variables and is thus also not able to use them. 

=======
To build the solver run:

make

=======
To run the solver:

./probSAT instance.cnf <seed>

=======
The solver accepts a series of parameters which can be displayed by typing:

./probSAT -h

=======

In-Memory API Usage (for C/C++ Integration)
-------------------------------------------

probSAT can be used as a library via its in-memory API, allowing you to solve SAT problems directly from your C/C++ code without file I/O.

**Function signature:**

```c
int probsat_solve_in_memory(
    int numVars,                // Number of variables
    int numClauses,             // Number of clauses
    int** clauses,              // Array of pointers to clauses (each clause is an array of ints, ending with 0)
    long long seed,             // Random seed
    long long maxRuns,          // Maximum number of tries
    long long maxFlips,         // Maximum number of flips per try
    ProbSatResult* result,      // Output struct for results
    int* initialAssignment      // (Optional) Initial assignment array (1/0), or NULL for random
);
```

**Result struct:**
```c
typedef struct {
    int sat;            // 1 for SAT, 0 for UNSAT
    int* assignment;    // Array of variable assignments (1 for true, 0 for false)
    int num_flips;      // Number of flips performed
    double solve_time;  // Time taken to solve (seconds)
    // More stats can be added as needed
} ProbSatResult;
```

**Minimal usage example:**
```c
#include "probSAT_inmem.c"

// Example: Solve (x1 OR ~x2) AND (~x1 OR x2)
int main() {
    int numVars = 2;
    int numClauses = 2;
    // Each clause is an array ending with 0
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

For more details, see comments in `probSAT_inmem.c`.