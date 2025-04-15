#!/bin/bash

# Run XFeat-SLAM with ORB features in Monocular mode with EuRoC dataset
# Based on XFORBMono_EuroC configuration from launch.json

# Set dataset variable - replace MH01 with desired sequence if needed
DATASET="MH01"

# Set environment variables
export USE_ORB=1
export Matcher_Debug=1

# Execute the monocular XFeat-SLAM with ORB features on EuRoC dataset
# cd "$(dirname "$0")/.."
./../Examples/XF/mono_euroc \
    ../Vocabulary/ORBvoc.txt \
    ../XFeatSLAM/examples/Monocular/EuRoC.yaml \
    ../datasets/${DATASET} \
    ../XFeatSLAM/examples/Monocular/EuRoC_TimeStamps/${DATASET}.txt 2>&1 | tee ../debug_output/xf_orb_debug_log_${DATASET}_$(date +%Y%m%d_%H%M%S).txt

echo "XFeat-SLAM with ORB features Monocular run with ${DATASET} completed" 