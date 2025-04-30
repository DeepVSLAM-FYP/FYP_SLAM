#include "FeatureExtractors/SIFTextractor.h"
#include <opencv2/imgproc.hpp>

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
    sift_->detectAndCompute(image, mask, kps, desc);

    // 2) Force every keypoint to octave = 0
    for (auto &kp : kps) {
        kp.octave = 0;
    }

    // 3) No laplacian area for SIFT—clear it
    vLap.clear();

    // 4) Build a single‐level pyramid so downstream stereo code can use pyr_[0]
    pyr_.resize(1);
    pyr_[0] = image;

    return static_cast<int>(kps.size());
}

}  // namespace ORB_SLAM3
