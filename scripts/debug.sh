# This script is used to set the debug flags for debugging SLAM code 

# Unset all - Don't touch these
unset DEBUG_FEAT
unset DEBUG_SLAM
unset SKIP_SLAM

# mono_euroc_pipelined_dummy.cc
unset DEBUG_KeypointVisualization

unset DEBUG_SearchByProjection
unset DEBUG_SearchByBoW
unset DEBUG_SearchForInitialization
unset DEBUG_SearchForInitializationDist
unset DEBUG_Fuse
unset DEBUG_SearchByProjectionKF
unset DEBUG_SearchByProjectionFrame
unset DEBUG_SearchBySim3
unset DEBUG_DescriptorDistance
unset DEBUG_ComputeThreeMaxima
unset DEBUG_DISTANCE
unset DEBUG_TrackT
unset DEBUG_TrackWithMM
unset DEBUG_TrackLM
unset DEBUG_NeedNewKeyFrameT
unset DEBUG_CreateNewKeyFrameT
unset DEBUG_SearchLocalPointsT
unset DEBUG_GrabImageT
unset DEBUG_UpdateLocalMapT
unset DEBUG_TrackLocalMap
unset DEBUG_MonocularInitializationT
unset DEBUG_MonocularInitializationShowMatchedCoords
unset USE_ORB

# Change here - uncomment the debug flags you want to set
export DEBUG_FEAT=1
# export DEBUG_SLAM=1
# export SKIP_SLAM=1  # Set this to skip SLAM initialization and processing (for feature extraction only)

# mono_euroc_pipelined_dummy.cc
# export DEBUG_KeypointVisualization=1    # Debug visualization of keypoints  

# ORBMatcher.cc
# export DEBUG_SearchByProjection=1
# export DEBUG_SearchByBoW=1
# export DEBUG_SearchForInitialization=1
# export DEBUG_SearchForInitializationDist=1
# export DEBUG_Fuse=1
# export DEBUG_SearchByProjectionKF=1
# export DEBUG_SearchByProjectionFrame=1
# export DEBUG_SearchBySim3=1
# export DEBUG_DescriptorDistance=1
# export DEBUG_ComputeThreeMaxima=1
# export DEBUG_DISTANCE=1

#Tracking.cc
# --- Main tracking process ---
# export DEBUG_TrackT=1                              # Debug information at the end of Track()
# export DEBUG_TrackWithMM=1                         # Debug tracking with motion model
# export DEBUG_TrackLM=1                             # Debug tracking of local map points
# export DEBUG_NeedNewKeyFrameT=1                    # Debug whether a new keyframe is needed
# export DEBUG_CreateNewKeyFrameT=1                  # Debug keyframe creation
# export DEBUG_SearchLocalPointsT=1                  # Debug local point searching process
# export DEBUG_TrackLocalMap=1                       # Debug tracking of local map points
# export DEBUG_UpdateLocalMapT=1                     # Debug local map updating process

# --- Frame grabbing ---
# export DEBUG_GrabImageT=1                          # Debug frame grabbing process

# --- Initialization ---
# export DEBUG_MonocularInitializationT=1            # Debug monocular initialization process
# export DEBUG_MonocularInitializationShowMatchedCoords=1  # Debug coordinates of matched points during initialization 

# Other
# export USE_ORB=1
