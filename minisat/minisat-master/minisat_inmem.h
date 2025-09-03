#ifndef MINISAT_INMEM_H
#define MINISAT_INMEM_H

#ifdef __cplusplus
extern "C" {
#endif

// Result structure for the in-memory API (similar to ProbSAT)
typedef struct {
    int sat;            // 1 for SAT, 0 for UNSAT
    int* assignment;    // Array of variable assignments (1 for true, 0 for false)
    int num_decisions;  // Number of decisions made (equivalent to flips in ProbSAT)
    double solve_time;  // Time taken to solve (seconds)
    char* error_message; // Error message if solving failed
} MiniSatResult;

// Main API function for in-memory solving
int minisat_solve_in_memory(
    int numVars,
    int numClauses,
    int** clauses,
    MiniSatResult* result,
    int* initialAssignment // array of 1/0 values, or NULL
);

// Utility function to free result memory
void minisat_free_result(MiniSatResult* result);

#ifdef __cplusplus
}
#endif

#endif // MINISAT_INMEM_H 