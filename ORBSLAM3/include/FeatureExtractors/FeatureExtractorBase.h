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
    virtual int operator()(const cv::Mat& image, cv::InputArray _mask,
                            std::vector<cv::KeyPoint>& _keypoints,
                            cv::Mat& descriptors,
                            std::vector<int>& vLappingArea) = 0;

    // --- scale-space info -----------------------------------------
    virtual int                 GetLevels()            const = 0;
    virtual float               GetScaleFactor()       const = 0;
    virtual const std::vector<float>&     GetScaleFactors()      const = 0;
    virtual const std::vector<float>&     GetInverseScaleFactors()   const = 0;
    virtual const std::vector<float>&     GetScaleSigmaSquares()       const = 0;
    virtual const std::vector<float>&     GetInverseScaleSigmaSquares()    const = 0;
    virtual const std::vector<cv::Mat>& GetImagePyramid() const = 0;

    // --- descriptor traits ----------------------------------------
    virtual int       descriptorType()   const = 0;   // OpenCV enum
    virtual int       descriptorLength() const = 0;   // bytes/floats
    virtual NormType  norm()             const = 0;   // Hamming or L2

};
} // namespace ORB_SLAM3
