/**
* File: Dummyextractor.cc
* Date: April 2025
* Description: Implementation of mock feature extractor
* License: see LICENSE.txt
*/

#include "FeatureExtractors/Dummy/Dummyextractor.h"
#include <fstream>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <opencv2/features2d.hpp>
#include "FeatureExtractors/Dummy/FeatureIO.h"  // Added include for FeatureIO

namespace fs = std::filesystem;

namespace ORB_SLAM3
{

DummyExtractor::DummyExtractor(int nfeatures, float scaleFactor, int nlevels, 
                             int iniThFAST, int minThFAST,
                             const std::string& keypointsDir,
                             const std::string& descriptorsDir,
                             const std::string& extractorType,
                             int descriptorType,
                             int descriptorSize)
    : nfeatures(nfeatures), scaleFactor(scaleFactor), nlevels(nlevels),
      iniThFAST(iniThFAST), minThFAST(minThFAST), 
      keypointsDirectory(keypointsDir), descriptorsDirectory(descriptorsDir),
      extractorType(extractorType), descriptorType(descriptorType), descriptorSize(descriptorSize)
{
    // Initialize scale parameters
    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);

    mvScaleFactor[0] = 1.0f;
    mvLevelSigma2[0] = 1.0f;
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i] = mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i] = mvScaleFactor[i]*mvScaleFactor[i];
    }

    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i] = 1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i] = 1.0f/mvLevelSigma2[i];
    }

    // Configure for specific extractor type
    configureForExtractorType();
}

void DummyExtractor::configureForExtractorType() 
{
    // Configure based on extractorType
    if(extractorType == "ORB") {
        descriptorType = CV_8U;
        descriptorSize = 32;
    } 
    else if(extractorType == "SIFT") {
        descriptorType = CV_32F;
        descriptorSize = 128;
    }
    else if(extractorType == "SURF") {
        descriptorType = CV_32F;
        descriptorSize = 64;
    }
    else if(extractorType == "SP") {
        descriptorType = CV_32F;
        descriptorSize = 256;
    }
    else if(extractorType == "XFEAT") {
        descriptorType = CV_32F;
        descriptorSize = 128;
    }
    // Default to ORB if the type is not recognized
    else {
        std::cerr << "Warning: Unrecognized extractor type '" << extractorType 
                  << "'. Defaulting to ORB configuration." << std::endl;
        extractorType = "ORB";
        descriptorType = CV_8U;
        descriptorSize = 32;
    }
}

int DummyExtractor::operator()(cv::InputArray _image, cv::InputArray _mask,
                              std::vector<cv::KeyPoint>& _keypoints,
                              cv::OutputArray _descriptors, std::vector<int> &vLappingArea)
{
    // Clear any previous keypoints and descriptors
    _keypoints.clear();
    
    // Check if we have image path encoded in vLappingArea
    std::string imagePath;
    if(vLappingArea.size() > 2) {
        // Reconstruct the image path from the encoded ASCII values
        for(size_t i = 2; i < vLappingArea.size(); i++) {
            if(vLappingArea[i] != 0) {
                imagePath += static_cast<char>(vLappingArea[i]);
            }
        }
    }
    
    if(imagePath.empty()) {
        std::cerr << "Error: No image path provided in vLappingArea. Cannot load keypoints/descriptors." << std::endl;
        return 0;
    }
    
    // Extract base filename without directory or extension
    std::string baseFilename = getBaseFilename(imagePath);
    
    // Load keypoints
    bool kptsLoaded = loadKeypoints(baseFilename, _keypoints);
    if(!kptsLoaded) {
        std::cerr << "Error: Failed to load keypoints for " << baseFilename << std::endl;
        return 0;
    }
    
    // Load descriptors
    cv::Mat descriptors;
    bool descLoaded = loadDescriptors(baseFilename, descriptors);
    if(!descLoaded) {
        std::cerr << "Error: Failed to load descriptors for " << baseFilename << std::endl;
        _keypoints.clear();
        return 0;
    }
    
    // Output the descriptors
    _descriptors.create(descriptors.rows, descriptors.cols, descriptors.type());
    descriptors.copyTo(_descriptors);
    
    return _keypoints.size();
}

std::string DummyExtractor::getBaseFilename(const std::string& filepath) 
{
    fs::path path(filepath);
    return path.stem().string();
}

bool DummyExtractor::loadKeypoints(const std::string& imageBaseName, std::vector<cv::KeyPoint>& keypoints) 
{
    // Ensure keypoints directory has a trailing slash if not empty
    std::string kpDir = keypointsDirectory.empty() ? "" : 
                        (keypointsDirectory.back() == '/' ? keypointsDirectory : keypointsDirectory + "/");
    
    // Full path to keypoints file
    std::string kptsFilePath = kpDir + imageBaseName + ".kpts";
    
    // Use FeatureIO to load keypoints
    bool success = FeatExtraction::FeatureIO::loadKeypoints(keypoints, kptsFilePath);
    
    if (!success) {
        std::cerr << "Cannot load keypoints file: " << kptsFilePath << std::endl;
    }
    
    return success;
}

bool DummyExtractor::loadDescriptors(const std::string& imageBaseName, cv::Mat& descriptors)
{
    // Ensure descriptors directory has a trailing slash if not empty
    std::string descDir = descriptorsDirectory.empty() ? "" : 
                         (descriptorsDirectory.back() == '/' ? descriptorsDirectory : descriptorsDirectory + "/");
    
    // Full path to descriptors file
    std::string descFilePath = descDir + imageBaseName + ".desc";
    
    // Use FeatureIO to load descriptors
    bool success = FeatExtraction::FeatureIO::loadDescriptors(descriptors, descFilePath);
    
    if (!success) {
        std::cerr << "Cannot load descriptors file: " << descFilePath << std::endl;
    }
    
    return success;
}

} // namespace ORB_SLAM3