/*
 * probSAT version SC13_v2
 * uses only break!
 * in case of 3-SAT implements pick and flip method without caching
 * Author: Adrian Balint
 *
 * Extended for in-memory CNF solving (API at bottom)
 */

#define _POSIX_C_SOURCE 199309L

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/times.h>
#include <string.h>
#include <limits.h>
#include <float.h>
#include <getopt.h>
#include <signal.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAXCLAUSELENGTH 10000 //maximum number of literals per clause //TODO: eliminate this limit
#define STOREBLOCK  20000
# undef LLONG_MAX
#define LLONG_MAX  9223372036854775807
#define BIGINT long long int

void (*initLookUpTable)() = NULL;
void (*pickAndFlipVar)() = NULL;

// --- Global variables (for CLI compatibility and in-memory API) ---
int numVars;
int numClauses;
int numLiterals;
char *atom;
int **clause;
int maxClauseSize;
int minClauseSize;
int *numOccurrence;
int **occurrence;
int maxNumOccurences = 0;
int numFalse;
int *falseClause;
int *whereFalse;
unsigned short *numTrueLit;
int *breaks;
int *critVar;
int bestVar;
double *probsBreak;
double *probs;
double cb;
double eps = 1.0;
int fct = 0;
int caching = 0;
FILE *fp;
char *fileName;
char* initAssignmentsFile = NULL;
BIGINT seed;
BIGINT maxRuns = LLONG_MAX;
BIGINT maxFlips = LLONG_MAX;
BIGINT flip;
float timeOut = FLT_MAX;
int printSol = 0;
int run;
double runTime;
double tryTime;
double solverTime;
double solverTime_s;
double totalSolverTime;
double totalSolverTime_s;
double loadingTime;
struct timespec start, end;
long ticks_per_second;
int bestNumFalse;
int cm_spec = 0, cb_spec = 0, fct_spec = 0, caching_spec = 0;

int* initialAssignmentPtr = NULL;

inline int abs(int a) {
    return (a < 0) ? -a : a;
}

void printFormulaProperties() {
    printf("\nc %-20s:  %s\n", "instance name", fileName);
    printf("c %-20s:  %d\n", "number of variables", numVars);
    printf("c %-20s:  %d\n", "number of literals", numLiterals);
    printf("c %-20s:  %d\n", "number of clauses", numClauses);
    printf("c %-20s:  %d\n", "max. clause length", maxClauseSize);
}

void printProbs() {
    int i;
    printf("c Probs values:\n");
    printf("c  ");
    for (i = 0; i <= 10; i++)
        printf(" %7i |", i);

    printf("\nc b");
    for (i = 0; i <= 10; i++) {
        if (probsBreak[i] != 0)
            printf(" %-6.5f |", probsBreak[i]);
    }
    printf("\n");
}

void printSolverParameters() {
    printf("\nc probSAT parameteres: \n");
    printf("c %-20s: %-20s\n", "using:", "only break");
    if (fct == 0)
        printf("c %-20s: %-20s\n", "using:", "polynomial function");
    else
        printf("c %-20s: %-20s\n", "using:", "exponential function");

    printf("c %-20s: %6.6f\n", "cb", cb);
    if (fct == 0) { //poly
        printf("c %-20s: %-20s\n", "function:", "probsBreak[break]*probsMake[make] = pow((eps + break), -cb);");
        printf("c %-20s: %6.6f\n", "eps", eps);
    } else { //exp
        printf("c %-20s: %-20s\n", "function:", "probsBreak[break]*probsMake[make] = pow(cb, -break);");
    }
    if (caching)
        printf("c %-20s: %-20s\n", "using:", "caching of break values");
    else
        printf("c %-20s: %-20s\n", "using:", "no caching of break values");
    //printProbs();
    printf("\nc general parameteres: \n");
    printf("c %-20s: %lli\n", "maxRuns", maxRuns);
    printf("c %-20s: %lli\n", "maxFlips", maxFlips);
    printf("c %-20s: %lli\n", "seed", seed);
    printf("c %-20s: \n", "-->Starting solver");
    fflush(stdout);
}

void printSolution() {
    int i;
    printf("v ");
    for (i = 1; i <= numVars; i++) {
        if (i % 21 == 0)
            printf("\nv ");
        if (atom[i] == 1)
            printf("%d ", i);
        else
            printf("%d ", -i);
    }
}

void readInitialAssignments() {
    if (initAssignmentsFile == NULL) {
        fprintf(stderr, "Initial assignments file not set!\n");
        exit(EXIT_FAILURE);
    }

    FILE* fp = fopen(initAssignmentsFile, "r");
    if (fp == NULL) {
        fprintf(stderr, "Error opening initial assignments file: %s\n", initAssignmentsFile);
        exit(EXIT_FAILURE);
    }

    int var, val;
    while (fscanf(fp, "%d %d", &var, &val) == 2) {
        if (var > 0 && var <= numVars) {
            atom[var] = (val == 1) ? 1 : 0;
        } else if (var < 0 && -var <= numVars) {
            atom[-var] = (val == 1) ? 0 : 1;
        } else {
            printf("Warning: variable index %d is out of range\n", var);
        }
    }
    fclose(fp);
}

static inline void printStatsEndFlip() {
    if (numFalse < bestNumFalse) {
        bestNumFalse = numFalse;
    }
}

static inline void allocateMemory() {
    numLiterals = numVars * 2;
    atom = (char*) malloc(sizeof(char) * (numVars + 1));
    clause = (int**) malloc(sizeof(int*) * (numClauses + 1));
    numOccurrence = (int*) malloc(sizeof(int) * (numLiterals + 1));
    occurrence = (int**) malloc(sizeof(int*) * (numLiterals + 1));
    critVar = (int*) malloc(sizeof(int) * (numClauses + 1));
    falseClause = (int*) malloc(sizeof(int) * (numClauses + 1));
    whereFalse = (int*) malloc(sizeof(int) * (numClauses + 1));
    numTrueLit = (unsigned short*) malloc(sizeof(unsigned short) * (numClauses + 1));
}

// --- SAT solving and helper functions (continued) ---

// === MISSING CORE FUNCTIONS FROM ORIGINAL probSAT.c ===

// Parses a DIMACS CNF file and sets up the clause/occurrence structures
static inline void parseFile() {
    int i, j;
    int lit, r;
    int clauseSize;
    int tatom;
    char c;
    long filePos;
    fp = NULL;
    fp = fopen(fileName, "r");
    if (fp == NULL) {
        fprintf(stderr, "c Error: Not able to open the file: %s\n", fileName);
        exit(-1);
    }
    // Scan header for numVars and numClauses
    for (;;) {
        c = fgetc(fp);
        if (c == 'c')
            do { c = fgetc(fp); } while ((c != '\n') && (c != EOF));
        else if (c == 'p') {
            if ((fscanf(fp, "%*s %d %d", &numVars, &numClauses)))
                break;
        } else {
            printf("c No parameter line found! Computing number of atoms and number of clauses from file!\n");
            r = fseek(fp, -1L, SEEK_CUR);
            if (r == -1) { fprintf(stderr, "c Error: Not able to seek in file: %s", fileName); exit(-1); }
            filePos = ftell(fp);
            if (r == -1) { fprintf(stderr, "c Error: Not able to obtain position in file: %s", fileName); exit(-1); }
            numVars = 0; numClauses = 0;
            for (; fscanf(fp, "%i", &lit) == 1;) {
                if (lit == 0) numClauses++;
                else { tatom = abs(lit); if (tatom > numVars) numVars = tatom; }
            }
            printf("c numVars: %d numClauses: %d\n", numVars, numClauses);
            r = fseek(fp, filePos, SEEK_SET);
            if (r == -1) { fprintf(stderr, "c Error: Not able to seek in file: %s", fileName); exit(-1); }
            break;
        }
    }
    allocateMemory();
    maxClauseSize = 0;
    minClauseSize = MAXCLAUSELENGTH;
    int *numOccurrenceT = (int*) malloc(sizeof(int) * (numLiterals + 1));
    int freeStore = 0;
    int *tempClause = 0;
    for (i = 0; i < numLiterals + 1; i++) { numOccurrence[i] = 0; numOccurrenceT[i] = 0; }
    for (i = 1; i <= numClauses; i++) {
        whereFalse[i] = -1;
        if (freeStore < MAXCLAUSELENGTH) {
            tempClause = (int*) malloc(sizeof(int) * STOREBLOCK);
            freeStore = STOREBLOCK;
        }
        clause[i] = tempClause;
        clauseSize = 0;
        do {
            r = fscanf(fp, "%i", &lit);
            if (lit != 0) {
                clauseSize++;
                *tempClause++ = lit;
                numOccurrenceT[numVars + lit]++;
            } else {
                *tempClause++ = 0;
            }
            freeStore--;
        } while (lit != 0);
        if (clauseSize > maxClauseSize) maxClauseSize = clauseSize;
        if (clauseSize < minClauseSize) minClauseSize = clauseSize;
    }
    for (i = 0; i < numLiterals + 1; i++) {
        occurrence[i] = (int*) malloc(sizeof(int) * (numOccurrenceT[i] + 1));
        occurrence[i][numOccurrenceT[i]] = 0;
        if (numOccurrenceT[i] > maxNumOccurences) maxNumOccurences = numOccurrenceT[i];
    }
    for (i = 1; i <= numClauses; i++) {
        j = 0;
        while ((lit = clause[i][j])) {
            occurrence[lit + numVars][numOccurrence[lit + numVars]++] = i;
            j++;
        }
        occurrence[lit + numVars][numOccurrence[lit + numVars]] = 0;
    }
    probs = (double*) malloc(sizeof(double) * (numVars + 1));
    breaks = (int*) malloc(sizeof(int) * (numVars + 1));
    free(numOccurrenceT);
    fclose(fp);
}

// Initializes the assignment and clause state for each try
static inline void init() {
    printf("[init] Entering init()\n"); fflush(stdout);
    ticks_per_second = sysconf(_SC_CLK_TCK);
    int i, j;
    int critLit = 0, lit;
    numFalse = 0;
    printf("[init] numClauses=%d, numVars=%d\n", numClauses, numVars); fflush(stdout);
    for (i = 1; i <= numClauses; i++) {
        numTrueLit[i] = 0;
        whereFalse[i] = -1;
    }
    printf("[init] Finished zeroing numTrueLit and whereFalse\n"); fflush(stdout);
    // In init(), always initialize breaks[i] = 0 for all i
    for (i = 1; i <= numVars; i++) {
        breaks[i] = 0;
    }
    if(initAssignmentsFile == NULL && initialAssignmentPtr == NULL){
        for (i = 1; i <= numVars; i++) {
            atom[i] = rand() % 2;
        }
    } else {
        printf("[init] Preserving provided initial assignment: ");
        for (int vi = 1; vi <= numVars; ++vi) printf("%d ", atom[vi]);
        printf("\n");
    }
    printf("[init] Finished initializing atom and breaks\n"); fflush(stdout);
    for (i = 1; i <= numClauses; i++) {
        j = 0;
        if (clause[i] == NULL) {
            printf("[init] clause[%d] is NULL!\n", i); fflush(stdout); continue; }
        while ((lit = clause[i][j])) {
            //if (abs(lit) < 1 || abs(lit) > numVars) {
                //printf("[init] lit=%d at clause[%d][%d] out of bounds!\n", lit, i, j); fflush(stdout); break; }
            if (atom[abs(lit)] == (lit > 0)) {
                numTrueLit[i]++;
                critLit = lit;
            }
            j++;
        }
        if (numTrueLit[i] == 1) {
            critVar[i] = abs(critLit);
            breaks[abs(critLit)]++;
        } else if (numTrueLit[i] == 0) {
            falseClause[numFalse] = i;
            whereFalse[i] = numFalse;
            numFalse++;
        }
    }
    printf("[init] Exiting init()\n"); fflush(stdout);
}

// Checks if the current assignment satisfies all clauses
static inline int checkAssignment() {
    int i, j;
    int sat, lit;
    for (i = 1; i <= numClauses; i++) {
        sat = 0;
        j = 0;
        while ((lit = clause[i][j])) {
            if (atom[abs(lit)] == (lit > 0))
                sat = 1;
            j++;
        }
        if (sat == 0)
            return 0;
    }
    return 1;
}

// Non-caching version of the variable flip logic
static inline void pickAndFlipNC() {
    extern int numVars;
    extern char* atom;
    int i, j;
    int bestVar;
    int rClause, tClause;
    rClause = falseClause[flip % numFalse];
    bestVar = abs(clause[rClause][0]);
    double randPosition;
    int lit;
    double sumProb = 0;
    int xMakesSat = 0;
    i = 0;
    while ((lit = clause[rClause][i])) {
        breaks[i] = 0;
        j=0;
        while ((tClause = occurrence[numVars - lit][j])){
            if (numTrueLit[tClause] == 1)
                breaks[i]++;
            j++;
        }
        probs[i] = probsBreak[breaks[i]];
        sumProb += probs[i];
        i++;
    }
    randPosition = (double) (rand()) / RAND_MAX * sumProb;
    for (i = i - 1; i != 0; i--) {
        sumProb -= probs[i];
        if (sumProb <= randPosition)
            break;
    }
    bestVar = abs(clause[rClause][i]);
    if (atom[bestVar])
        xMakesSat = -bestVar;
    else
        xMakesSat = bestVar;
    atom[bestVar] = 1 - atom[bestVar];
    i = 0;
    while ((tClause = occurrence[xMakesSat + numVars][i])) {
        if (numTrueLit[tClause] == 0) {
            falseClause[whereFalse[tClause]] = falseClause[--numFalse];
            whereFalse[falseClause[numFalse]] = whereFalse[tClause];
            whereFalse[tClause] = -1;
        }
        numTrueLit[tClause]++;
        i++;
    }
    i = 0;
    while ((tClause = occurrence[numVars - xMakesSat][i])) {
        if (numTrueLit[tClause] == 1) {
            falseClause[numFalse] = tClause;
            whereFalse[tClause] = numFalse;
            numFalse++;
        }
        numTrueLit[tClause]--;
        i++;
    }
}

// Caching version of the variable flip logic
static inline void pickAndFlip() {
    extern int numVars;
    extern char* atom;
    int var;
    int bestVar;
    int rClause = falseClause[flip % numFalse];
    double sumProb = 0.0;
    double randPosition;
    int i, j;
    int tClause;
    int xMakesSat;
    i = 0;
    while ((var = abs(clause[rClause][i]))) {
        probs[i] = probsBreak[breaks[var]];
        sumProb += probs[i];
        i++;
    }
    randPosition = (double) (rand()) / RAND_MAX * sumProb;
    for (i = i - 1; i != 0; i--) {
        sumProb -= probs[i];
        if (sumProb <= randPosition)
            break;
    }
    bestVar = abs(clause[rClause][i]);
    if (atom[bestVar] == 1)
        xMakesSat = -bestVar;
    else
        xMakesSat = bestVar;
    atom[bestVar] = 1 - atom[bestVar];
    i = 0;
    while ((tClause = occurrence[xMakesSat + numVars][i])) {
        if (numTrueLit[tClause] == 0) {
            falseClause[whereFalse[tClause]] = falseClause[--numFalse];
            whereFalse[falseClause[numFalse]] = whereFalse[tClause];
            whereFalse[tClause] = -1;
            critVar[tClause] = abs(xMakesSat);
            breaks[bestVar]++;
        } else {
            if (numTrueLit[tClause] == 1) {
                breaks[critVar[tClause]]--;
            }
        }
        numTrueLit[tClause]++;
        i++;
    }
    i = 0;
    while ((tClause = occurrence[numVars - xMakesSat][i])) {
        if (numTrueLit[tClause] == 1) {
            falseClause[numFalse] = tClause;
            whereFalse[tClause] = numFalse;
            numFalse++;
            breaks[bestVar]--;
        } else if (numTrueLit[tClause] == 2) {
            j = 0;
            while ((var = abs(clause[tClause][j]))) {
                if (((clause[tClause][j] > 0) == atom[abs(var)])) {
                    critVar[tClause] = var;
                    breaks[var]++;
                    break;
                }
                j++;
            }
        }
        numTrueLit[tClause]--;
        i++;
    }
}

// Start the CPU timer
void start_timer(struct timespec* start) {
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, start) == -1) {
        perror("clock_gettime");
        exit(EXIT_FAILURE);
    }
}

// Calculate the elapsed CPU time
double elapsed_time(struct timespec* start) {
    struct timespec end;
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end) == -1) {
        perror("clock_gettime");
        exit(EXIT_FAILURE);
    }
    double elapsed = (end.tv_sec - start->tv_sec) + (end.tv_nsec - start->tv_nsec) / 1e9;
    return elapsed;
}

// Start wall clock timer
void start_clock_timer(struct timespec *start) {
    if (clock_gettime(CLOCK_MONOTONIC, start) == -1) {
        perror("clock_gettime");
        exit(EXIT_FAILURE);
    }
}

// Calculate the elapsed wall clock time
double elapsed_clock_time(struct timespec *start) {
    struct timespec end;
    if (clock_gettime(CLOCK_MONOTONIC, &end) == -1) {
        perror("clock_gettime");
        exit(EXIT_FAILURE);
    }
    double elapsed = (end.tv_sec - start->tv_sec) + (end.tv_nsec - start->tv_nsec) / 1e9;
    return elapsed;
}

double elapsed_seconds(void) {
    double answer;
    static struct tms prog_tms;
    static long prev_times = 0;
    (void) times(&prog_tms);
    answer = ((double) (((long) prog_tms.tms_utime) - prev_times)) / ((double) ticks_per_second);
    prev_times = (long) prog_tms.tms_utime;
    return answer;
}

double elapsed_seconds_solver_time(void) {
    double answer;
    static struct tms prog_tms;
    static long prev_times = 0;
    (void) times(&prog_tms);
    answer = ((double) (((long) prog_tms.tms_utime) - prev_times)) / ((double) ticks_per_second);
    prev_times = (long) prog_tms.tms_utime;
    return answer;
}

double elapsed_seconds_loading_time(void) {
    double answer;
    static struct tms prog_tms;
    static long prev_times = 0;
    (void) times(&prog_tms);
    answer = ((double) (((long) prog_tms.tms_utime) - prev_times)) / ((double) ticks_per_second);
    prev_times = (long) prog_tms.tms_utime;
    return answer;
}

static inline void printEndStatistics() {
    printf("\nc EndStatistics:\n");
    printf("c %-30s: %-9lli\n", "numFlips", flip);
    printf("c %-30s: %-9i\n", "numTries", run);
    printf("c %-30s: %-8.2f\n", "avg. flips/variable", (double) flip / (double) numVars);
    printf("c %-30s: %-8.2f\n", "avg. flips/clause", (double) flip / (double) numClauses);
    printf("c %-30s: %-8.0f\n", "flips/sec", (double) flip / tryTime);
    printf("c %-30s: %-8.4f\n", "CPU Time", tryTime);
    printf("c %-30s: %-8.4f\n", "Loading Time", loadingTime);
    printf("c %-30s: %-8.4f\n", "Solver Time", totalSolverTime);
    printf("c %-30s: %-8.4f\n", "Clock Time", totalSolverTime_s);
}

static inline void printUsage() {
    printf("\n----------------------------------------------------------\n");
    printf("probSAT version SC13.2\n");
    printf("Authors: Adrian Balint\n");
    printf("Ulm University - Institute of Theoretical Computer Science \n");
    printf("2013\n");
    printf("----------------------------------------------------------\n\n");
    printf("\nUsage of probSAT:\n");
    printf("./probSAT [options] <DIMACS CNF instance> [<seed>]\n");
    printf("\nprobSAT options:\n");
    printf("which function to use:\n");
    printf("--fct <0,1> : 0 =  polynomial; 1 = exponential [default = 0]\n");
    printf("--eps <double_value> : eps>0 (only valid when --fct 0)[default = 1.0]\n");
    printf("which constant to use in the functions:\n");
    printf("--cb <double_value> : constant for break [default = k dependet]\n");
    printf("\nFurther options:\n");
    printf("--caching <0,1>, -c<0,1>  : use caching of break values \n");
    printf("--runs <int_value>, -r<int_value>  : maximum number of runs \n");
    printf("--maxflips <int_value> , -m<int_value>: number of flips per run \n");
    printf("--printSolution, -a : output assignment\n");
    printf("--help, -h : output this help\n");
    printf("----------------------------------------------------------\n\n");
}

void initPoly() {
    int i;
    probsBreak = (double*) malloc(sizeof(double) * (maxNumOccurences + 1));
    for (i = 0; i <= maxNumOccurences; i++) {
        probsBreak[i] = pow((eps + i), -cb);
    }
}

void initExp() {
    int i;
    probsBreak = (double*) malloc(sizeof(double) * (maxNumOccurences + 1));
    for (i = 0; i <= maxNumOccurences; i++) {
        probsBreak[i] = pow(cb, -i);
    }
}

void parseParameters(int argc, char *argv[]) {
    //define the argument parser
    static struct option long_options[] =
            { {"initFile", required_argument, 0, 'i'}, { "fct", required_argument, 0, 'f' }, { "caching", required_argument, 0, 'c' }, { "eps", required_argument, 0, 'e' }, { "cb", required_argument, 0, 'b' }, { "runs", required_argument, 0, 't' }, { "maxflips", required_argument, 0, 'm' }, { "printSolution", no_argument, 0, 'a' }, { "help", no_argument, 0, 'h' }, { 0, 0, 0, 0 } };

    while (optind < argc) {
        int index = -1;
        struct option * opt = 0;
        int result = getopt_long(argc, argv, "i:f:e:c:b:t:m:ah", long_options, &index); //
        if (result == -1)
            break; /* end of list */
        switch (result) {
        case 'i':
            initAssignmentsFile = optarg;
            if (initAssignmentsFile != NULL) {
                printf("Initial assignments file: %s\n", initAssignmentsFile);
            } else {
                printf("Initial assignments file is not set or NULL\n");
            }
            break;
        case 'h':
            printUsage();
            exit(0);
            break;
        case 'c':
            caching = atoi(optarg);
            caching_spec = 1;
            break;
        case 'f':
            fct = atoi(optarg);
            fct_spec = 1;
            break;
        case 'e':
            eps = atof(optarg);
            if (eps <= 0) {
                printf("\nERROR: eps should >0!!!\n");
                exit(0);
            }
            break;
        case 'b':
            cb = atof(optarg);
            cb_spec = 1;
            break;
        case 't': //maximum number of tries to solve the problems within the maxFlips
            maxRuns = atoi(optarg);
            break;
        case 'm': //maximum number of flips to solve the problem
            maxFlips = atoi(optarg);
            break;
        case 'a': //print assignment for variables at the end
            printSol = 1;
            break;
        case 0: /* all parameter that do not */
            /* appear in the optstring */
            opt = (struct option *) &(long_options[index]);
            printf("'%s' was specified.", opt->name);
            if (opt->has_arg == required_argument)
                printf("Arg: <%s>", optarg);
            printf("\n");
            break;
        default:
            printf("parameter not known!\n");
            printUsage();
            exit(0);
            break;
        }
    }
    if (optind == argc) {
        printf("ERROR: Parameter String not correct!\n");
        printUsage();
        exit(0);
    }
    fileName = *(argv + optind);

    if (argc > optind + 1) {
        seed = atoi(*(argv + optind + 1));
        if (seed == 0)
            printf("c there might be an error in the command line or is your seed 0?");
    } else
        seed = time(0);
}

// Update the signal handler to have the correct signature for C/C++
void handle_interrupt(int signum) {
    (void)signum;
    printf("\nc caught signal... exiting\n ");
    tryTime = elapsed_seconds();
    solverTime = elapsed_seconds_solver_time();
    loadingTime = elapsed_seconds_loading_time();
    printf("\ns UNKNOWN best(%d) (%-15.5fsec)\n", bestNumFalse, tryTime);
    printf("\ns UNKNOWN best(%d) (%-15.5fsec) solver Time\n", bestNumFalse, solverTime);
    printf("\ns UNKNOWN best(%d) (%-15.5fsec) loading Time\n", bestNumFalse, loadingTime);
    printEndStatistics();
    fflush(NULL);
    exit(-1);
}

void setupSignalHandler() {
    signal(SIGTERM, handle_interrupt);
    signal(SIGINT, handle_interrupt);
    signal(SIGQUIT, handle_interrupt);
    signal(SIGABRT, handle_interrupt);
    // SIGKILL cannot be caught or handled, so remove it
}

void setupParameters() {
    if (!caching_spec) {
        if (maxClauseSize <= 3){
            pickAndFlipVar = pickAndFlipNC; //no caching of the break values in case of 3SAT
            caching =0;
        }
        else{
            pickAndFlipVar = pickAndFlip; //cache the break values for other k-SAT
            caching = 1;
        }
    }
    else{
        if (caching)
            pickAndFlipVar = pickAndFlip; //cache the break values for other k-SAT
        else
            pickAndFlipVar = pickAndFlipNC; //no caching of the break values in case of 3SAT
    }
    if (!cb_spec) {
        if (maxClauseSize <= 3) {
            cb = 2.06;
            eps = 0.9;

        } else if (maxClauseSize <= 4)
            //cb = 2.85;
            cb = 3.0;
        else if (maxClauseSize <= 5)
            //cb = 3.7;
            cb = 3.88;
        else if (maxClauseSize <= 6)
            //cb = 5.1;
            cb =4.6;
        else
            //cb = 5.4;
            cb = 4.6;
    }
    if (!fct_spec) {
        if (maxClauseSize < 4)
            fct = 0;
        else
            fct = 1;
    }
    if (fct == 0)
        initLookUpTable = initPoly;
    else
        initLookUpTable = initExp;
}

// --- In-memory API and result struct will remain at the bottom ---

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int sat;            // 1 for SAT, 0 for UNSAT
    int* assignment;    // Array of variable assignments (1 for true, 0 for false)
    int num_flips;      // Number of flips performed
    double solve_time;  // Time taken to solve (seconds)
    // Add more stats as needed
} ProbSatResult;

// --- Main solving logic refactored from main() ---
// This function is called by both the CLI and the in-memory API
int probsat_run() {
    
    run = 0;
    runTime = 0.;
    double totalTime = 0.;
    solverTime = 0.;
    solverTime_s = 0.;

    struct timespec start_tms;
    struct timespec start_clock;
    loadingTime = elapsed_seconds_loading_time();
    
    start_timer(&start_tms);
    printFormulaProperties();
    setupParameters(); //call only after parsing file or in-memory setup!!!
    initLookUpTable(); //Initialize the look up table
    setupSignalHandler();
    printSolverParameters();
    srand(seed);

    //initial variable assignment if provided
    if (initAssignmentsFile != NULL) {
        readInitialAssignments();
    }    
    
    loadingTime = elapsed_seconds_loading_time();
    loadingTime = elapsed_time(&start_tms);
    start_timer(&start_tms);
    start_clock_timer(&start_clock);

    for (run = 0; run < maxRuns; run++) {
        init();
        
        bestNumFalse = numClauses;
        for (flip = 0; flip < maxFlips; flip++) {
            if (numFalse == 0)
                break;
            pickAndFlipVar();
            printStatsEndFlip(); //update bestNumFalse
        }
        tryTime = elapsed_seconds();
        totalTime += tryTime;
        
        if (numFalse == 0) {
            solverTime = elapsed_time(&start_tms);
            totalSolverTime += solverTime;
            solverTime_s = elapsed_clock_time(&start_clock);
            totalSolverTime_s += solverTime_s;
            if (!checkAssignment()) {
                fprintf(stderr, "c ERROR the assignment is not valid!");
                printf("c UNKNOWN");
                return 0;
            } else {
                printEndStatistics();
                printf("s SATISFIABLE\n");
                if (printSol == 1)
                    printSolution();
                return 10;
            }
        } else {
            printf("c UNKNOWN best(%4d) current(%4d) (%-15.5fsec)\n", bestNumFalse, numFalse, tryTime);
            printf("c UNKNOWN best(%4d) current(%4d) (%-15.5fsec) solver Time\n", bestNumFalse, numFalse, totalSolverTime);
            printf("c UNKNOWN best(%4d) current(%4d) (%-15.5fsec) solver (s) Time\n", bestNumFalse, numFalse, totalSolverTime_s);
        }
    }
    printEndStatistics();
    if (maxRuns > 1) {
        printf("c %-30s: %-8.3fsec\n", "Mean time per run", totalTime / (double) run);
        printf("c %-30s: %-8.3fsec\n", "Mean solver time per run", totalSolverTime / (double) run);
        printf("c %-30s: %-8.3fsec\n", "Mean solver (s) time per run", totalSolverTime_s / (double) run);
    }
    return 0;
}

int probsat_solve_in_memory(
    int numVars_,
    int numClauses_,
    int** clauses_,
    long long seed_,
    long long maxRuns_,
    long long maxFlips_,
    ProbSatResult* result,
    int* initialAssignment // NEW: array of 1/0 values, or NULL
) {
    printf("Entered probsat_solve_in_memory: numVars=%d, numClauses=%d\n", numVars_, numClauses_);
    fflush(stdout);
    
    // Validate input parameters
    if (numVars_ <= 0 || numClauses_ <= 0 || clauses_ == NULL || result == NULL) {
        printf("ERROR: Invalid input parameters\n");
        return 0;
    }
    // Set up global variables as if parseFile() had run
    numVars = numVars_;
    numClauses = numClauses_;
    maxRuns = maxRuns_;
    maxFlips = maxFlips_;
    seed = seed_;
    printSol = 0; // don't print solution to stdout

    // Allocate memory as parseFile() would
    printf("Allocating memory...\n");
    fflush(stdout);
    numLiterals = numVars * 2;
    atom = (char*) malloc(sizeof(char) * (numVars + 1));
    clause = (int**) malloc(sizeof(int*) * (numClauses + 1));
    numOccurrence = (int*) malloc(sizeof(int) * (numLiterals + 1));
    occurrence = (int**) malloc(sizeof(int*) * (numLiterals + 1));
    critVar = (int*) malloc(sizeof(int) * (numClauses + 1));
    falseClause = (int*) malloc(sizeof(int) * (numClauses + 1));
    whereFalse = (int*) malloc(sizeof(int) * (numClauses + 1));
    numTrueLit = (unsigned short*) malloc(sizeof(unsigned short) * (numClauses + 1));
    
    // Check if all allocations succeeded
    if (!atom || !clause || !numOccurrence || !occurrence || !critVar || !falseClause || !whereFalse || !numTrueLit) {
        printf("ERROR: Memory allocation failed\n");
        return 0;
    }
    printf("Memory allocation successful\n");
    fflush(stdout);

    // --- Set maxClauseSize and minClauseSize ---
    printf("Printing first 5 clauses after receiving CNF in probsat_solve_in_memory:\n");
    for (int i = 1; i <= numClauses && i <= 5; ++i) {
        printf("Clause %d: ", i);
        int k = 0;
        while (clauses_[i][k] != 0 && k < 20) { // print up to 20 literals
            printf("%d ", clauses_[i][k]);
            ++k;
        }
        printf("0\n");
    }
    for (int i = 1; i <= 5 && i <= numClauses; ++i) {
        printf("[C] clauses_[%d] = %p\n", i, (void*)clauses_[i]);
    }
    fflush(stdout);
    printf("Calculating clause sizes...\n");
    fflush(stdout);
    maxClauseSize = 0;
    minClauseSize = MAXCLAUSELENGTH;
    for (int i = 1; i <= numClauses; ++i) {
        if (clauses_[i] == NULL) {
            printf("ERROR: Clause %d is NULL\n", i);
            return 0;
        }
        if (clauses_[i][0] == 0) {
            printf("ERROR: Clause %d is empty (just null terminator)!\n", i);
            return 0;
        }
        int len = 0;
        while (clauses_[i][len] != 0) ++len;
        if (len > maxClauseSize) maxClauseSize = len;
        if (len < minClauseSize) minClauseSize = len;
    }
    printf("Max clause size: %d, Min clause size: %d\n", maxClauseSize, minClauseSize);
    fflush(stdout);

    // Print first and last clause pointers and contents
    printf("First clause pointer: %p\n", (void*)clauses_[1]);
    printf("First clause literals: ");
    for (int k = 0; k < 10 && clauses_[1][k] != 0; ++k) printf("%d ", clauses_[1][k]);
    printf("0\n");
    printf("Last clause pointer: %p\n", (void*)clauses_[numClauses]);
    printf("Last clause literals: ");
    for (int k = 0; k < 10 && clauses_[numClauses][k] != 0; ++k) printf("%d ", clauses_[numClauses][k]);
    printf("0\n");
    fflush(stdout);

    // --- Fill clause pointers ---
    printf("Filling clause pointers...\n");
    fflush(stdout);
    for (int i = 1; i <= numClauses; ++i) {
        clause[i] = clauses_[i];
    }
    printf("Clause pointers filled\n");
    fflush(stdout);

    // --- Fill numOccurrence and occurrence arrays ---
    printf("Processing occurrence arrays...\n");
    fflush(stdout);
    int *numOccurrenceT = (int*) malloc(sizeof(int) * (numLiterals + 1));
    if (!numOccurrenceT) {
        printf("ERROR: Failed to allocate numOccurrenceT\n");
        return 0;
    }
    for (int i = 0; i < numLiterals + 1; i++) { numOccurrence[i] = 0; numOccurrenceT[i] = 0; }
    for (int i = 1; i <= numClauses; i++) {
        int j = 0;
        int lit;
        while ((lit = clause[i][j])) {
            int idx = numVars + lit;
            /*
            if (idx < 0 || idx > numLiterals) {
                printf("[occurrence arrays] Out of bounds: i=%d, j=%d, lit=%d, idx=%d (numVars=%d, numLiterals=%d)\n", i, j, lit, idx, numVars, numLiterals);
                fflush(stdout);
                break;
            }
            */
            numOccurrenceT[idx]++;
            j++;
        }
    }
    printf("Occurrence arrays first pass done\n");
    fflush(stdout);
    maxNumOccurences = 0;
    for (int i = 0; i < numLiterals + 1; i++) {
        occurrence[i] = (int*) malloc(sizeof(int) * (numOccurrenceT[i] + 1));
        occurrence[i][numOccurrenceT[i]] = 0;
        if (numOccurrenceT[i] > maxNumOccurences) maxNumOccurences = numOccurrenceT[i];
    }
    for (int i = 1; i <= numClauses; i++) {
        int j = 0;
        int lit;
        while ((lit = clause[i][j])) {
            occurrence[lit + numVars][numOccurrence[lit + numVars]++] = i;
            j++;
        }
        occurrence[lit + numVars][numOccurrence[lit + numVars]] = 0;
    }
    free(numOccurrenceT);

    // --- Allocate probs and breaks arrays ---
    probs = (double*) malloc(sizeof(double) * (numVars + 1));
    breaks = (int*) malloc(sizeof(int) * (numVars + 1));
    probsBreak = (double*) malloc(sizeof(double) * (maxClauseSize + 1));

    // --- Use in-memory initial assignment if provided ---
    if (initialAssignment != NULL) {
        initialAssignmentPtr = initialAssignment;
        printf("[ProbSAT DEBUG] Using provided initial assignment: ");
        for (int i = 1; i <= numVars; ++i) {
            atom[i] = initialAssignment[i-1]; // Fix: use 0-based indexing for input array
            printf("%d ", atom[i]);
        }
        printf("\n");
        initAssignmentsFile = NULL; // Ensure file-based init is not used
    } else {
        initialAssignmentPtr = NULL;
        // fallback: file-based or random assignment (handled in solver)
    }

    // Start timer
    struct timespec start_time, end_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    // Now run the main solving logic
    int sat = probsat_run();

    // End timer
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    double solve_time = (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_nsec - start_time.tv_nsec) / 1e9;

    // Fill the result struct
    result->sat = sat;
    result->assignment = (int*)malloc(sizeof(int) * numVars);
    for (int i = 1; i <= numVars; ++i) {
        result->assignment[i-1] = atom[i];
    }
    result->num_flips = (int)flip; // or another stat if available
    result->solve_time = solve_time;
    // Add more stats as needed

    // Free memory
    free(atom);
    free(clause);
    free(numOccurrence);
    for (int i = 0; i < numLiterals + 1; i++) {
        free(occurrence[i]);
    }
    free(occurrence);
    free(critVar);
    free(falseClause);
    free(whereFalse);
    free(numTrueLit);
    free(probs);
    free(breaks);
    free(probsBreak);

    return sat; // 1 for SAT, 0 for UNSAT, etc.
}

#ifdef __cplusplus
}
#endif






#ifdef __cplusplus
}
#endif

// --- CLI main() remains unchanged, but now calls probsat_run() after input parsing ---
#ifdef PROBSAT_CLI_MAIN
int main(int argc, char *argv[]) {
    ticks_per_second = sysconf(_SC_CLK_TCK);
    parseParameters(argc, argv);
    parseFile();
    // Now call the refactored main logic
    return probsat_run();
}
#endif
// --- End of probSAT_inmem.c --- 