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
```

- The `mdd/` folder contains all C++ (and Python) source and header files related to MDD construction and manipulation.
- `main.cpp` demonstrates usage and provides tests for the MDD classes.
- `.gitignore` and `README.md` are for project management and documentation.

## Building the Project

To build the project, use the following command:

```
g++ -std=c++17 -O2 -o main mdd/MDDConstructor.cpp mdd/MDD.cpp mdd/MDDNode.cpp
```

Replace `main` with your desired output executable name, and ensure all required source files are listed.

## Usage
- The code is designed for research and educational purposes related to pathfinding and decision diagrams.
- See the source files for more details on usage and implementation. 