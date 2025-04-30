// include/FeatureExtractorBase.h
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace ORB_SLAM3 {

enum class NormType { HAMMING, L2 };

class FeatureExtractor {
public:
    virtual ~FeatureExtractor() = default;

    // --- main call -------------------------------------------------
    virtual void operator()(const cv::Mat& image,
                            std::vector<cv::KeyPoint>& kps,
                            cv::Mat& descriptors,
                            std::vector<int>& vLappingArea) const = 0;

    // --- scale-space info -----------------------------------------
    virtual int                 levels()            const = 0;
    virtual float               scaleFactor()       const = 0;
    virtual const std::vector<float>&     scaleFactors()      const = 0;
    virtual const std::vector<float>&     invScaleFactors()   const = 0;
    virtual const std::vector<float>&     levelSigma2()       const = 0;
    virtual const std::vector<float>&     invLevelSigma2()    const = 0;

    // --- descriptor traits ----------------------------------------
    virtual int       descriptorType()   const = 0;   // OpenCV enum
    virtual int       descriptorLength() const = 0;   // bytes/floats
    virtual NormType  norm()             const = 0;   // Hamming or L2

    // --- optional: reuse the pyramid for stereo -------------------
    virtual const std::vector<cv::Mat>& imagePyramid() const = 0;
};
} // namespace ORB_SLAM3
