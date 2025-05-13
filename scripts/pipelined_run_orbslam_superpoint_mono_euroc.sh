#!/bin/bash

# Run ORB-SLAM3 in Monocular pipelined mode with real-time SuperPoint DPU feature extractor and EuRoC dataset
# Based on the mono_euroc_superpoint implementation

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Set dataset variable - replace V101 with desired sequence if needed
DATASET=${1:-"V101"}  # Use first argument or default to V101

# Set SuperPoint model path (default to compiled_SP_by_H.xmodel)
MODEL_PATH=${2:-"$PROJECT_ROOT/Thirdparty/super_point_vitis/compiled_SP_by_H.xmodel"}

# Set number of threads (default 4)
NUM_THREADS=${3:-4}

# Print paths for debugging
echo "Running from script directory: $SCRIPT_DIR"
echo "Project root: $PROJECT_ROOT"
echo "Dataset: $DATASET"
echo "SuperPoint model: $MODEL_PATH"
echo "Num threads: $NUM_THREADS"

# Enable debug mode
export DEBUG_SLAM=1

# source debug.sh if exists
if [ -f "$SCRIPT_DIR/debug.sh" ]; then
    source "$SCRIPT_DIR/debug.sh"
fi

# Execute the pipelined SuperPoint ORB-SLAM3 with EuRoC dataset
"$PROJECT_ROOT/Examples/ORB/mono_euroc_superpoint" \
    "$PROJECT_ROOT/Vocabulary/dpu_bovisa_sp_h.fbow" \
    "$PROJECT_ROOT/ORBSLAM3/Examples/Monocular/EuRoC_Dummy.yaml" \
    "$PROJECT_ROOT/datasets/${DATASET}" \
    "$PROJECT_ROOT/ORBSLAM3/Examples/Monocular/EuRoC_TimeStamps/${DATASET}.txt" \
    "$MODEL_PATH" \
    "$NUM_THREADS" \
    "$PROJECT_ROOT/Trajectories/ORB_SuperPoint/${DATASET}/" 2>&1 | tee "$PROJECT_ROOT/debug_output/o_p_sp_debug_log_${DATASET}_$(date +%Y%m%d_%H%M%S).txt"

echo "ORB-SLAM3 Monocular Pipelined SuperPoint Extractor run with ${DATASET} completed" 