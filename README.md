# MDDConstructor Project

This project implements a Multi-value Decision Diagram (MDD) constructor in C++. It provides functionality to build MDDs for grid-based pathfinding problems, supporting features like obstacle handling, shortest path calculation, and flexible time-step management.

## Overview

This repository provides code and data for constructing and manipulating Multi-value Decision Diagrams (MDDs) for grid-based pathfinding, as used in research on multi-agent pathfinding and related problems. It is intended as a supplement to the associated research paper.

## Repository Structure

```
MAPF_LNS_SAT/
├── mdd/                  # C++ and Python source/header files for MDD construction
│   ├── MDDConstructor.cpp
│   ├── MDDConstructor.h
│   ├── MDD.cpp
│   ├── MDD.h
│   ├── MDDNode.cpp
│   ├── MDDNode.h
│   └── MDD.py
├── cnf/                  # CNF construction and related files
├── minisat/              # MiniSAT integration (see minisat/README.md)
├── probSAT-master/       # probSAT SAT solver (CLI and C/C++ API)
├── mapf-map/             # Grid map files for experiments
├── mapf-scen-even/       # Scenario files for experiments
├── main.cpp              # Example usage and tests for the MDD classes
├── SATSolverManager.h    # SAT solver management and integration
├── SATSolverManager.cpp  # SAT solver management implementation
├── .gitignore            # Standard ignore rules for C++ projects
├── LICENSE               # MIT License
└── README.md             # Project documentation and build instructions
```

- **mdd/**: Core C++/Python code for MDD construction and manipulation.
- **cnf/**: CNF construction and related utilities.
- **minisat/**: MiniSAT integration with wrappers, tests, and documentation (see `minisat/README.md`).
- **probSAT-master/**: Third-party SAT solver (see its README for details).
- **mapf-map/**, **mapf-scen-even/**: Datasets (maps and scenarios) for experiments.
- **SATSolverManager.h/cpp**: SAT solver management and integration for both ProbSAT and MiniSAT.
- **main.cpp**: Example usage and tests for the MDD classes.

## Building the Project

To build the main C++ components, use:

```
g++ -std=c++17 -O2 -o main mdd/MDDConstructor.cpp mdd/MDD.cpp mdd/MDDNode.cpp
```

Replace `main` with your desired output executable name, and ensure all required source files are listed. You may need to adjust the command if you use additional components.

## Usage
- The code is designed for research and educational purposes related to pathfinding and decision diagrams.
- See the source files for more details on usage and implementation.
- Example usage is provided in `main.cpp`.

## Datasets
- **mapf-map/** contains grid maps used in experiments.
- **mapf-scen-even/** contains scenario files for benchmarking.
- You can add your own maps/scenarios following the same format.

## Results and Logging
- All experiment logs and results (CSV logs, intermediate files, etc.) are written to the `data/` directory in the repository.
- This directory is included in the repository (not in .gitignore) to ensure reproducibility and easy sharing of results.
- Log files include:
  - `data/solver_log.csv` (overall run summaries)
  - `data/solver_log_timesteps.csv` (per-timestep iteration logs)
  - `data/solver_log_collisions.csv` (per-collision iteration logs)
- You can analyze these logs with Python, R, Excel, or any data analysis tool.

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## How to Cite
**TODO: Add BibTeX citation for the associated paper here.**

## Author / Contact
**TODO: Add author name(s) and contact information here.**

## SAT Solvers

This project supports multiple SAT solvers:

- **ProbSAT**: Stochastic local search solver (in `probSAT-master/`)
- **MiniSAT**: CDCL solver (in `minisat/`)

See `minisat/README.md` for MiniSAT integration details and usage examples.

## Acknowledgements
- The probSAT SAT solver is included under its own license in `probSAT-master/`.
- The MiniSAT solver is included under its own license in `minisat/minisat-master/`.
- This project was developed for research purposes. If you use this code or data, please cite the associated paper (see above). 