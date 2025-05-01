#!/bin/bash
# This script is used to set the debug flags for debugging SLAM code 

# Unset all - Don't touch these
unset DEBUG_FEAT
unset DEBUG_SLAM
unset DEBUG_SearchByProjection
unset DEBUG_SearchByBoW
unset DEBUG_SearchForInitialization
unset DEBUG_Fuse
unset DEBUG_SearchByProjectionKF
unset DEBUG_SearchByProjectionFrame
unset DEBUG_SearchBySim3
unset DEBUG_DescriptorDistance
unset DEBUG_ComputeThreeMaxima
unset DEBUG_DISTANCE
unset USE_ORB

# Change here - uncomment the debug flags you want to set
# export DEBUG_FEAT=1
# export DEBUG_SLAM=1
# export DEBUG_SearchByProjection=1
# export DEBUG_SearchByBoW=1
# export DEBUG_SearchForInitialization=1
# export DEBUG_Fuse=1
# export DEBUG_SearchByProjectionKF=1
# export DEBUG_SearchByProjectionFrame=1
# export DEBUG_SearchBySim3=1
# export DEBUG_DescriptorDistance=1
# export DEBUG_ComputeThreeMaxima=1
# export DEBUG_DISTANCE=1
# export USE_ORB=1