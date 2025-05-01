#include "FeatureExtractors/SIFTextractor.h"
#include <opencv2/imgproc.hpp>
#include <iostream>

namespace ORB_SLAM3 {

SIFTextractor::SIFTextractor(int nfeatures) {
    // Build a standard SIFT detector / descriptor
    sift_ = cv::SIFT::create(nfeatures);
    // We only ever need one image level, so leave sf_, isf_, etc. as {1.0}
    // pyr_ will be built on demand in operator()
}

int SIFTextractor::operator()(const cv::Mat& image, cv::InputArray mask,
                              std::vector<cv::KeyPoint>& kps,
                              cv::Mat& desc,
                              std::vector<int>& vLap) {
    // 1) Detect and compute SIFT descriptors at original resolution
    // 1) Detect all keypoints
    std::vector<cv::KeyPoint> allKeypoints;
    sift_->detect(image, allKeypoints, mask);

    // Filter to keep only octave 0 keypoints
    kps.clear();

    int nKeypoints = allKeypoints.size();
    
    if(std::getenv("DEBUG_FEAT") != nullptr && strcmp(std::getenv("DEBUG_FEAT"), "1") == 0) 
    {
            std::cout << "nKeypoints: " << nKeypoints << std::endl;
    }

    for (int i = 0; i < nKeypoints; ++i) {
        auto& kp = allKeypoints[i];
        int octave = kp.octave & 255;  // lower byte = octave index
        if (octave == 0){
        kp.octave = 0;  // set octave to 0 for all 
        kps.push_back(kp);
        }
        
    }

    // Compute descriptors only for octave 0 keypoints
    sift_->compute(image, kps, desc);

    vLap.clear();

    // 4) Build a single‐level pyramid
    pyr_.resize(1);
    pyr_[0] = image;

    return static_cast<int>(kps.size());
}

}  // namespace ORB_SLAM3
