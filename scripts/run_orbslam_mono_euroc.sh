#!/bin/bash

# Run ORB-SLAM3 in Monocular mode with EuRoC dataset
# Based on Mono_EuroC configuration from launch.json

# Set dataset variable - replace MH01 with desired sequence if needed
DATASET="MH01"

# Execute the monocular ORB-SLAM3 with EuRoC dataset
# cd "$(dirname "$0")/.."
./../Examples/ORB/monoORB_euroc \
    ../Vocabulary/ORBvoc.txt \
    ../ORBSLAM3/Examples/Monocular/EuRoC.yaml \
    ../datasets/${DATASET} \
    ../ORBSLAM3/Examples/Monocular/EuRoC_TimeStamps/${DATASET}.txt \
    ../Trajectories/ORB/${DATASET}/ 2>&1 | tee ../debug_output/orbslam_debug_log_${DATASET}_$(date +%Y%m%d_%H%M%S).txt

echo "ORB-SLAM3 Monocular run with ${DATASET} completed" 