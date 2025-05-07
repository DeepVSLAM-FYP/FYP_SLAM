/**
 * File: DummyAdapter.h
 * Date: April 2025
 * Description: Mock feature extractor adapter using pre-computed features
 * License: see LICENSE.txt
 */

#ifndef DUMMY_FEATURE_EXTRACTOR_H
#define DUMMY_FEATURE_EXTRACTOR_H

#include "FeatureExtractorBase.h"
#include "FeatureExtractors/Dummy/Dummyextractor.h"

namespace ORB_SLAM3 {

class DummyAdapter : public AdapterInterface {
public:
    DummyAdapter(int nFeatures = 1000, float scaleFactor = 1.2f, 
                int nLevels = 8, int iniThFAST = 20, int minThFAST = 7,
                const std::string& keypointsDir = "", 
                const std::string& descriptorsDir = "",
                const std::string& extractorType = "ORB",
                int descriptorType = CV_8U,
                int descriptorSize = 32);
    
    virtual ~DummyAdapter();

    virtual int extract(const cv::Mat& image, 
                        std::vector<cv::KeyPoint>& keypoints,
                        cv::Mat& descriptors,
                        const std::string& imagePath = "") override;

    virtual int getDescriptorType() const override;
    
    virtual int getDescriptorSize() const override;
    

    virtual std::string getExtractorName() const override;

    void setKeypointsDirectory(const std::string& dir);
    
    void setDescriptorsDirectory(const std::string& dir);

private:
    // The actual mock extractor
    ORB_SLAM3::DummyExtractor* mExtractor;
    
    // Parameters
    int mNumFeatures;
    float mScaleFactor;
    int mNumLevels;
    int mIniThFAST;
    int mMinThFAST;
    std::string mExtractorType;
    int mDescriptorType;
    int mDescriptorSize;
    
    // Data directories
    std::string mKeypointsDir;
    std::string mDescriptorsDir;
};

} // namespace FeatExtraction

#endif // DUMMY_FEATURE_EXTRACTOR_H