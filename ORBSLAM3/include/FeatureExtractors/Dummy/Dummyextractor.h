/**
* File: Dummyextractor.h
* Date: April 2025
* Description: Mock feature extractor that loads pre-computed keypoints and descriptors
* License: see LICENSE.txt
*/

#ifndef DUMMYEXTRACTOR_H
#define DUMMYEXTRACTOR_H

#include <vector>
#include <list>
#include <map>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM3
{

class DummyExtractor
{
public:
    DummyExtractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST,
                 const std::string& keypointsDir = "",
                 const std::string& descriptorsDir = "",
                 const std::string& extractorType = "ORB",
                 int descriptorType = CV_8U,
                 int descriptorSize = 32);

    ~DummyExtractor() {}

    // Main feature extraction method
    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

    // Configuration helpers
    void configureForExtractorType();
    void SetKeypointsDirectory(const std::string& dir) { keypointsDirectory = dir; }
    void SetDescriptorsDirectory(const std::string& dir) { descriptorsDirectory = dir; }

    // Accessor methods
    int inline GetLevels() const { return nlevels; }
    float inline GetScaleFactor() const { return scaleFactor; }
    int inline GetInitThFAST() const { return iniThFAST; }
    int inline GetMinThFAST() const { return minThFAST; }
    int inline GetDescriptorType() const { return descriptorType; }
    int inline GetDescriptorSize() const { return descriptorSize; }
    std::string inline GetExtractorType() const { return extractorType; }

    std::vector<float> inline GetScaleFactors() const { return mvScaleFactor; }
    std::vector<float> inline GetInverseScaleFactors() const { return mvInvScaleFactor; }
    std::vector<float> inline GetScaleSigmaSquares() const { return mvLevelSigma2; }
    std::vector<float> inline GetInverseScaleSigmaSquares() const { return mvInvLevelSigma2; }

protected:
    // File access methods
    std::string getBaseFilename(const std::string& filepath);
    bool loadKeypoints(const std::string& imagePath, std::vector<cv::KeyPoint>& keypoints);
    bool loadDescriptors(const std::string& imagePath, cv::Mat& descriptors);

    // Parameters
    int nfeatures;
    float scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;
    std::string extractorType;
    int descriptorType;
    int descriptorSize;

    std::string keypointsDirectory;
    std::string descriptorsDirectory;

    // Storage for scale factors and sigma values
    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
    
    // Cache for loaded keypoints and descriptors
    std::map<std::string, std::vector<cv::KeyPoint>> keypointsCache;
    std::map<std::string, cv::Mat> descriptorsCache;

    // Image pyramid (for API compatibility)
    std::vector<cv::Mat> mvImagePyramid;

    // Features per level calculation
    std::vector<int> mnFeaturesPerLevel;

    // For pattern sampling
    std::vector<int> umax;
};

} //namespace ORB_SLAM3

#endif // DUMMYEXTRACTOR_H