/**
 * File: DummyAdapter.cpp
 * Date: April 2025
 * Description: Implementation of mock feature extractor adapter
 * License: see LICENSE.txt
 */

#include "FeatureExtractors/Dummy/DummyAdapter.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <filesystem>

namespace ORB_SLAM3 {

DummyAdapter::DummyAdapter(int nFeatures, float scaleFactor, 
                          int nLevels, int iniThFAST, int minThFAST,
                          const std::string& keypointsDir, 
                          const std::string& descriptorsDir,
                          const std::string& extractorType,
                          int descriptorType,
                          int descriptorSize) 
    : mNumFeatures(nFeatures), mScaleFactor(scaleFactor), 
      mNumLevels(nLevels), mIniThFAST(iniThFAST), mMinThFAST(minThFAST),
      mKeypointsDir(keypointsDir), mDescriptorsDir(descriptorsDir),
      mExtractorType(extractorType), mDescriptorType(descriptorType),
      mDescriptorSize(descriptorSize) {
    
    // Create the mock extractor with provided parameters
    mExtractor = new ORB_SLAM3::DummyExtractor(
        nFeatures, scaleFactor, nLevels, iniThFAST, minThFAST,
        keypointsDir, descriptorsDir, extractorType, descriptorType, descriptorSize
    );
}

DummyAdapter::~DummyAdapter() {
    if (mExtractor) {
        delete mExtractor;
        mExtractor = nullptr;
    }
}

int DummyAdapter::extract(const cv::Mat& image, 
                         std::vector<cv::KeyPoint>& keypoints,
                         cv::Mat& descriptors,
                         const std::string& imagePath) {
    // Check if we have a valid image path
    if (imagePath.empty()) {
        std::cerr << "DummyAdapter requires an image path to find corresponding .kpts and .desc files." << std::endl;
        return 0;
    }
    
    // Check if image is grayscale, convert if not
    cv::Mat grayImage;
    if (image.channels() == 3) {
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    } else {
        grayImage = image;
    }
    
    // Create the vLappingArea vector with encoded imagePath
    // First two elements are preserved for original lapping area use
    // Elements from index 2 onward will contain ASCII values of imagePath characters
    std::vector<int> vLappingArea(2 + imagePath.length(), 0);
    
    // Fill in the imagePath characters as ASCII values
    for (size_t i = 0; i < imagePath.length(); i++) {
        vLappingArea[i + 2] = static_cast<int>(imagePath[i]);
    }
    
    // Call the mock extractor with the encoded imagePath
    int featuresExtracted = mExtractor->operator()(grayImage, cv::noArray(), keypoints, descriptors, vLappingArea);
    
    if (featuresExtracted == 0) {
        std::cerr << "Warning: No features extracted for image " << imagePath << std::endl;
    }
    
    return featuresExtracted;
}

int DummyAdapter::getDescriptorType() const {
    return mExtractor->GetDescriptorType();
}

int DummyAdapter::getDescriptorSize() const {
    return mExtractor->GetDescriptorSize();
}

std::string DummyAdapter::getExtractorName() const {
    return "DUMMY_" + mExtractor->GetExtractorType();
}

void DummyAdapter::setKeypointsDirectory(const std::string& dir) {
    mKeypointsDir = dir;
    mExtractor->SetKeypointsDirectory(dir);
}

void DummyAdapter::setDescriptorsDirectory(const std::string& dir) {
    mDescriptorsDir = dir;
    mExtractor->SetDescriptorsDirectory(dir);
}

} // namespace FeatExtraction