#!/bin/bash

# Run ORB-SLAM3 in Monocular pipelined mode with Dummy feature extractor and EuRoC dataset
# Based on the mono_euroc_pipelined_dummy implementation

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Set dataset variable - replace MH01,V101 with desired sequence if needed
DATASET=${1:-"V101"}  # Use first argument or default to MH01

# Set feature directory - where to load features from
FEATURE_DIR=${2:-"$PROJECT_ROOT/datasets/feature_outputs/V101_h/V101"}  # Default to SP_H directory
# FEATURE_DIR=${2:-"$PROJECT_ROOT/datasets/feature_outputs/SP_H"}  # Default to SP_TF directory
# Print paths for debugging
echo "Running from script directory: $SCRIPT_DIR"
echo "Project root: $PROJECT_ROOT"
echo "Dataset: $DATASET"
echo "Feature directory: $FEATURE_DIR"

# Enable debug mode
export DEBUG_SLAM=1

# source debug.sh
source "$SCRIPT_DIR/debug.sh"

# Execute the pipelined dummy ORB-SLAM3 with EuRoC dataset
"$PROJECT_ROOT/Examples/ORB/mono_euroc_pipelined_dummy" \
    "$PROJECT_ROOT/Vocabulary/dpu_bovisa_sp_h.fbow" \
    "$PROJECT_ROOT/ORBSLAM3/Examples/Monocular/EuRoC_Dummy.yaml" \
    "$PROJECT_ROOT/datasets/${DATASET}" \
    "$PROJECT_ROOT/ORBSLAM3/Examples/Monocular/EuRoC_TimeStamps/${DATASET}.txt" \
    "$FEATURE_DIR" \
    "$PROJECT_ROOT/Trajectories/ORB_Dummy/${DATASET}/" 2>&1 | tee "$PROJECT_ROOT/debug_output/o_p_d_debug_log_${DATASET}_$(date +%Y%m%d_%H%M%S).txt"

echo "ORB-SLAM3 Monocular Pipelined Dummy Extractor run with ${DATASET} completed" 

# FEATURE_DIR=${2:-"$PROJECT_ROOT/datasets/feature_outputs/SP_TF"}  # Default to SP_H directory