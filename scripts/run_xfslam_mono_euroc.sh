#!/bin/bash

# Run XFeat-SLAM in Monocular mode with EuRoC dataset
# Based on XFMono_EuroC configuration from launch.json

# Set dataset variable - replace MH01 with desired sequence if needed
DATASET="MH01"

# Set environment variables
export Matcher_Debug=1

# Execute the monocular XFeat-SLAM with EuRoC dataset
# cd "$(dirname "$0")/.."
./../Examples/XF/mono_euroc \
    ../Vocabulary/ORBvoc.txt \
    ../XFeatSLAM/examples/Monocular/EuRoC.yaml \
    ../datasets/${DATASET} \
    ../XFeatSLAM/examples/Monocular/EuRoC_TimeStamps/${DATASET}.txt 2>&1 | tee ../debug_output/xf_xf_debug_log_${DATASET}_$(date +%Y%m%d_%H%M%S).txt

echo "XFeat-SLAM Monocular run with ${DATASET} completed" 