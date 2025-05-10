//utils/FeatureExtractorTypes.h
#pragma once
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


namespace ORB_SLAM3 {

enum class FeatureExtractorType {
    ORB,
    SUPERPOINT,
    DUMMY,
    CUSTOM
    // Add more extractor types as needed
};

// Input queue item for the feature extractor pipeline
struct InputQueueItem {
    InputQueueItem() : index(0), timestamp(0.0) {}
    InputQueueItem(
        int idx, double ts, 
        const std::string &fname, const cv::Mat &img)
        : index(idx), timestamp(ts), filename(fname), image(img) {}
    
    int index;              // Image index in sequence
    double timestamp;       // Timestamp of the image
    std::string filename;   // Original filename path
    cv::Mat image;          // Image data
};

// Result queue item containing extracted features
struct ResultQueueItem {
    ResultQueueItem() : index(0), timestamp(0.0) {}
    ResultQueueItem(
        int idx, double ts, 
        const std::string &fname, const cv::Mat &img)
        : index(idx), timestamp(ts), filename(fname), image(img) {}

    int index;                      // Image index in sequence
    double timestamp;               // Timestamp of the image
    std::string filename;           // Original filename path
    cv::Mat image;                  // Image data
    std::vector<cv::KeyPoint> keypoints;  // Extracted keypoints
    cv::Mat descriptors;            // Feature descriptors
    std::vector<int> lappingArea;   // For compatibility with ORB extractor
};

// enum class NormType {
//     HAMMING,  // For binary descriptors like ORB
//     L2        // For floating-point descriptors like SIFT, SURF, etc.
// };

} // namespace ORB_SLAM3