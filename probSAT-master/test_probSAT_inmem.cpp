// test_probSAT_inmem.cpp
// Example C++ test for the in-memory API of probSAT_inmem.c
//
// Build with:
//   g++ -O2 -std=c++11 test_probSAT_inmem.cpp probSAT_inmem.c -lm -o test_probSAT_inmem
//
// This test creates a simple SAT instance in memory and solves it using the C API.

#include <iostream>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <fstream>

// Extern C API from probSAT_inmem.c
extern "C" {
    typedef struct {
        int sat;            // 1 for SAT, 0 for UNSAT
        int* assignment;    // Array of variable assignments (1 for true, 0 for false)
        int num_flips;      // Number of flips performed
        double solve_time;  // Time taken to solve (seconds)
    } ProbSatResult;

    int probsat_solve_in_memory(
        int numVars_,
        int numClauses_,
        int** clauses_,
        long long seed_,
        long long maxRuns_,
        long long maxFlips_,
        ProbSatResult* result,
        int* initialAssignment // NEW: array of 1/0 values, or NULL
    );
    // Expose the global variable for initial assignment file
    extern char* initAssignmentsFile;
}

int main() {
    std::cout << "Entered main()" << std::endl;
    // Example SAT instance:
    // (x1 OR x2)
    // (NOT x1 OR x2)
    // (x1 OR NOT x2)
    // This is satisfiable: x1=1, x2=1
    const int numVars = 2;
    const int numClauses = 3;

    // Each clause is a null-terminated array of ints (DIMACS style)
    int clause1[] = { 1, 2, 0 };    // x1 OR x2
    int clause2[] = { -1, 2, 0 };   // NOT x1 OR x2
    int clause3[] = { 1, -2, 0 };   // x1 OR NOT x2

    // Array of pointers to clauses (1-based for probSAT)
    std::vector<int*> clauses(numClauses + 1);
    clauses[1] = clause1;
    clauses[2] = clause2;
    clauses[3] = clause3;


    // Prepare result struct
    ProbSatResult result;
    std::memset(&result, 0, sizeof(result));

    // Call the solver
    long long seed = 42; // Fixed seed for reproducibility
    long long maxTries = 100;
    long long maxFlips = 1000;
    int initialAssignment[3] = {0, 1, 1}; // 1-based: x1=1, x2=1

    // Debug prints before calling the solver
    std::cout << "About to call probsat_solve_in_memory" << std::endl;
    std::cout << "clauses.data() = " << clauses.data() << std::endl;
    for (int i = 1; i <= numClauses; ++i) {
        std::cout << "Clause " << i << ": ";
        for (int j = 0; clauses[i][j] != 0; ++j) {
            std::cout << clauses[i][j] << " ";
        }
        std::cout << "0" << std::endl;
    }
    std::cout << "initialAssignment = " << initialAssignment << std::endl;
    for (int i = 1; i <= numVars; ++i) {
        std::cout << "initialAssignment[" << i << "] = " << initialAssignment[i] << std::endl;
    }

    int sat = probsat_solve_in_memory(numVars, numClauses, clauses.data(), seed, maxTries, maxFlips, &result, initialAssignment);

    // Print result
    if (sat == 10 || result.sat == 10) {
        std::cout << "SATISFIABLE\nAssignment:";
        for (int i = 0; i < numVars; ++i) {
            std::cout << " x" << (i+1) << "=" << result.assignment[i];
        }
        std::cout << "\nFlips: " << result.num_flips << ", Time: " << result.solve_time << "s\n";
    } else {
        std::cout << "UNSAT or UNKNOWN\n";
    }

    // Free assignment array
    free(result.assignment);
    return 0;
} 