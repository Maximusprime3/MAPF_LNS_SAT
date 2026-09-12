#!/bin/bash

# MAPF LNS SAT Experiment Runner
# Usage: ./run_experiment.sh [config_file]
# If no config file is specified, uses lns/run_config/default_config.txt

CONFIG_FILE=${1:-"lns/run_config/default_config.txt"}

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file '$CONFIG_FILE' not found!"
    echo "Usage: ./run_experiment.sh [config_file]"
    echo "Available configs:"
    echo "  lns/run_config/default_config.txt"
    echo "  lns/run_config/experiment_100_agents.txt"
    echo "  lns/run_config/experiment_200_agents.txt"
    echo "  lns/run_config/experiment_probSAT.txt"
    echo "Example: ./run_experiment.sh lns/run_config/experiment_200_agents.txt"
    exit 1
fi

# Read configuration from file
echo "Loading configuration from: $CONFIG_FILE"
echo "=========================================="

# Source the config file
source "$CONFIG_FILE"

# Validate required parameters
if [ -z "$MAP_FILE" ] || [ -z "$SCENARIO_FILE" ] || [ -z "$NUM_AGENTS" ] || [ -z "$SCENARIO_INDEX" ] || [ -z "$SOLVER" ] || [ -z "$SEED" ] || [ -z "$LOG_FILE" ]; then
    echo "Error: Missing required configuration parameters!"
    echo "Required: MAP_FILE, SCENARIO_FILE, NUM_AGENTS, SCENARIO_INDEX, SOLVER, SEED, LOG_FILE"
    exit 1
fi

# Display configuration
echo "Configuration:"
echo "  Map file: $MAP_FILE"
echo "  Scenario file: $SCENARIO_FILE"
echo "  Number of agents: $NUM_AGENTS"
echo "  Scenario index: $SCENARIO_INDEX"
echo "  Solver: $SOLVER"
echo "  Seed: $SEED"
echo "  Log file: $LOG_FILE"
echo "=========================================="

# Check if executable exists
if [ ! -f "./main_crude_lns_helpers" ]; then
    echo "Error: main_crude_lns_helpers executable not found!"
    echo "Please compile the project first with: make -f Makefile_lns_helpers"
    exit 1
fi

# Check if map file exists
if [ ! -f "$MAP_FILE" ]; then
    echo "Error: Map file '$MAP_FILE' not found!"
    exit 1
fi

# Check if scenario file exists
if [ ! -f "$SCENARIO_FILE" ]; then
    echo "Error: Scenario file '$SCENARIO_FILE' not found!"
    exit 1
fi

# Run the experiment
echo "Starting experiment..."
echo "Command: ./main_crude_lns_helpers $MAP_FILE $SCENARIO_FILE $NUM_AGENTS $SCENARIO_INDEX $SOLVER $SEED"
echo "Output will be saved to: $LOG_FILE"
echo "=========================================="

# Execute the command and save output
./main_crude_lns_helpers "$MAP_FILE" "$SCENARIO_FILE" "$NUM_AGENTS" "$SCENARIO_INDEX" "$SOLVER" "$SEED" > "$LOG_FILE" 2>&1

# Check exit status
if [ $? -eq 0 ]; then
    echo "Experiment completed successfully!"
else
    echo "Experiment failed with exit code $?"
fi

# Show last 50 lines of log
echo "=========================================="
echo "Last 50 lines of output:"
echo "=========================================="
tail -n 50 "$LOG_FILE"

echo "=========================================="
echo "Full log saved to: $LOG_FILE"
echo "To view full log: cat $LOG_FILE"
echo "To follow log in real-time: tail -f $LOG_FILE"
