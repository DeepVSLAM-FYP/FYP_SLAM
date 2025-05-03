
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
unset DEBUG_TrackT
unset DEBUG_NeedNewKeyFrameT
unset DEBUG_CreateNewKeyFrameT
unset DEBUG_SearchLocalPointsT
unset DEBUG_GrabImageT
unset DEBUG_UpdateLocalMapT
unset DEBUG_MonocularInitializationT
unset USE_ORB

# Change here - uncomment the debug flags you want to set
export DEBUG_FEAT=1
export DEBUG_SLAM=1

# ORBMatcher.cc
export DEBUG_SearchByProjection=1
export DEBUG_SearchByBoW=1
export DEBUG_SearchForInitialization=1
# export DEBUG_Fuse=1
# export DEBUG_SearchByProjectionKF=1
# export DEBUG_SearchByProjectionFrame=1
# export DEBUG_SearchBySim3=1
# export DEBUG_DescriptorDistance=1
# export DEBUG_ComputeThreeMaxima=1
export DEBUG_DISTANCE=1

#Tracking.cc
export DEBUG_TrackT=1
export DEBUG_NeedNewKeyFrameT=1
export DEBUG_CreateNewKeyFrameT=1
export DEBUG_SearchLocalPointsT=1
export DEBUG_GrabImageT=1
export DEBUG_UpdateLocalMapT=1
export DEBUG_MonocularInitializationT=1 # prints out the initialization process

# Other
# export USE_ORB=1
