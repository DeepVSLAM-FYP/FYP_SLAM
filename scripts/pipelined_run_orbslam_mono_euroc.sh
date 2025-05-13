#!/bin/bash

# Run ORB-SLAM3 in Monocular pipelined mode with EuRoC dataset
# Based on the mono_euroc_pipelined implementation

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Set dataset variable - replace MH01,V101 with desired sequence if needed
DATASET=${1:-"MH01"}  # Use first argument or default to MH01

# Execute the pipelined monocular ORB-SLAM3 with EuRoC dataset
export DEBUG_SLAM=1

# Print paths for debugging if needed
echo "Running from script directory: $SCRIPT_DIR"
echo "Project root: $PROJECT_ROOT"
echo "Dataset: $DATASET"

# source debug.sh
source "$SCRIPT_DIR/debug.sh"

"$PROJECT_ROOT/Examples/ORB/mono_euroc_pipelined" \
    "$PROJECT_ROOT/Vocabulary/orb_mur.fbow" \
    "$PROJECT_ROOT/ORBSLAM3/Examples/Monocular/EuRoC.yaml" \
    "$PROJECT_ROOT/datasets/${DATASET}" \
    "$PROJECT_ROOT/ORBSLAM3/Examples/Monocular/EuRoC_TimeStamps/${DATASET}.txt" \
    "$PROJECT_ROOT/Trajectories/ORB_Pipelined/${DATASET}/" 2>&1 | tee "$PROJECT_ROOT/debug_output/orbslam_pipelined_debug_log_${DATASET}_$(date +%Y%m%d_%H%M%S).txt"

echo "ORB-SLAM3 Monocular Pipelined run with ${DATASET} completed"