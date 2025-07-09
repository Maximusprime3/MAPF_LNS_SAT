#ifndef PROBSAT_INMEM_H
#define PROBSAT_INMEM_H

#ifdef __cplusplus
extern "C" {
#endif

// Constants
#define MAXCLAUSELENGTH 10000
#define STOREBLOCK  20000
#undef LLONG_MAX
#define LLONG_MAX  9223372036854775807
#define BIGINT long long int

// Result structure for the in-memory API
typedef struct {
    int sat;            // 1 for SAT, 0 for UNSAT
    int* assignment;    // Array of variable assignments (1 for true, 0 for false)
    int num_flips;      // Number of flips performed
    double solve_time;  // Time taken to solve (seconds)
} ProbSatResult;

// Main API function for in-memory solving
int probsat_solve_in_memory(
    int numVars_,
    int numClauses_,
    int** clauses_,
    long long seed_,
    long long maxRuns_,
    long long maxFlips_,
    ProbSatResult* result,
    int* initialAssignment // array of 1/0 values, or NULL
);

// Internal function used by the API
int probsat_run(void);

// Utility functions (if needed)
void start_timer(struct timespec* start);
double elapsed_time(struct timespec* start);
void start_clock_timer(struct timespec *start);
double elapsed_clock_time(struct timespec *start);
double elapsed_seconds(void);
double elapsed_seconds_solver_time(void);
double elapsed_seconds_loading_time(void);

// Parameter setup functions
void parseParameters(int argc, char *argv[]);
void setupParameters(void);
void setupSignalHandler(void);

// Initialization functions
void initPoly(void);
void initExp(void);

// Solution handling
void printSolution(void);
void readInitialAssignments(void);

// Statistics and properties
void printFormulaProperties(void);
void printProbs(void);
void printSolverParameters(void);

// Signal handling
void handle_interrupt(int signum);

#ifdef __cplusplus
}
#endif

#endif // PROBSAT_INMEM_H 