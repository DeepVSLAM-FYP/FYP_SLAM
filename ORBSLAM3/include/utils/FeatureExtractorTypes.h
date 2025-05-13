//utils/FeatureExtractorTypes.h
#pragma once
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "utils/SuperPointTypes.h"

namespace ORB_SLAM3 {

enum class FeatureExtractorType {
    ORB,
    SUPERPOINT,
    DUMMY,
    CUSTOM
    // Add more extractor types as needed
};

// Use definitions from global namespace
using InputQueueItem = ::InputQueueItem;
using ResultQueueItem = ::ResultQueueItem;

} // namespace ORB_SLAM3