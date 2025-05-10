/**
 * File: FeatureIO.cpp
 * Date: April 2025
 * Description: Implementation of utilities for saving and loading feature keypoints and descriptors
 * License: see LICENSE.txt
 */

#include "FeatureExtractors/Dummy/FeatureIO.h"
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <cstring>

namespace ORB_SLAM3 {

// Helper functions for directory operations in C++14
bool createDirectories(const std::string& dirPath) {
    if (dirPath.empty())
        return true;

    size_t pos = 0;
    do {
        pos = dirPath.find_first_of("/\\", pos + 1);
        std::string subdir = dirPath.substr(0, pos);
        
        if (!subdir.empty()) {
            #ifdef _WIN32
                if (mkdir(subdir.c_str()) != 0 && errno != EEXIST) {
                    return false;
                }
            #else
                if (mkdir(subdir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0 && errno != EEXIST) {
                    return false;
                }
            #endif
        }
    } while (pos != std::string::npos);
    return true;
}

std::string getParentPath(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) {
        return "";
    }
    return path.substr(0, pos);
}

bool pathExists(const std::string& path) {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}

bool FeatureIO::saveFeatures(const std::vector<cv::KeyPoint>& keypoints,
                            const cv::Mat& descriptors,
                            const std::string& keypointsPath,
                            const std::string& descriptorsPath) {
    // Create directory if it doesn't exist
    std::string keypointsDir = getParentPath(keypointsPath);
    std::string descriptorsDir = getParentPath(descriptorsPath);
    
    try {
        if (!keypointsDir.empty() && !pathExists(keypointsDir)) {
            createDirectories(keypointsDir);
        }
        if (!descriptorsDir.empty() && !pathExists(descriptorsDir)) {
            createDirectories(descriptorsDir);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error creating directories: " << e.what() << std::endl;
        return false;
    }
    
    bool kpSuccess = saveKeypoints(keypoints, keypointsPath);
    bool descSuccess = saveDescriptors(descriptors, descriptorsPath);
    
    return kpSuccess && descSuccess;
}

bool FeatureIO::saveKeypoints(const std::vector<cv::KeyPoint>& keypoints, 
                             const std::string& filePath) {
    std::ofstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Unable to open file for writing keypoints: " << filePath << std::endl;
        return false;
    }
    
    // Save number of keypoints
    size_t numKeypoints = keypoints.size();
    file.write(reinterpret_cast<const char*>(&numKeypoints), sizeof(numKeypoints));
    
    // Save each keypoint
    for (const auto& kp : keypoints) {
        file.write(reinterpret_cast<const char*>(&kp.pt.x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&kp.pt.y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&kp.size), sizeof(float));
        file.write(reinterpret_cast<const char*>(&kp.angle), sizeof(float));
        file.write(reinterpret_cast<const char*>(&kp.response), sizeof(float));
        file.write(reinterpret_cast<const char*>(&kp.octave), sizeof(int));
        file.write(reinterpret_cast<const char*>(&kp.class_id), sizeof(int));
    }
    
    file.close();
    return true;
}

bool FeatureIO::saveDescriptors(const cv::Mat& descriptors, 
                               const std::string& filePath) {
    if (descriptors.empty()) {
        std::cerr << "No descriptors to save" << std::endl;
        return false;
    }
    
    std::ofstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Unable to open file for writing descriptors: " << filePath << std::endl;
        return false;
    }
    
    // Save matrix dimensions and type
    int rows = descriptors.rows;
    int cols = descriptors.cols;
    int type = descriptors.type();
    
    file.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
    file.write(reinterpret_cast<const char*>(&cols), sizeof(cols));
    file.write(reinterpret_cast<const char*>(&type), sizeof(type));
    
    // Save the matrix data
    if (descriptors.isContinuous()) {
        file.write(reinterpret_cast<const char*>(descriptors.data), 
                   descriptors.total() * descriptors.elemSize());
    } else {
        for (int i = 0; i < descriptors.rows; ++i) {
            file.write(reinterpret_cast<const char*>(descriptors.ptr(i)), 
                      descriptors.cols * descriptors.elemSize());
        }
    }
    
    file.close();
    return true;
}

bool FeatureIO::loadFeatures(std::vector<cv::KeyPoint>& keypoints,
                            cv::Mat& descriptors,
                            const std::string& keypointsPath,
                            const std::string& descriptorsPath) {
    bool kpSuccess = loadKeypoints(keypoints, keypointsPath);
    bool descSuccess = loadDescriptors(descriptors, descriptorsPath);
    
    return kpSuccess && descSuccess;
}

bool FeatureIO::loadKeypoints(std::vector<cv::KeyPoint>& keypoints, 
                             const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Unable to open file for reading keypoints: " << filePath << std::endl;
        return false;
    }
    
    // Clear the output vector
    keypoints.clear();
    
    // Read number of keypoints
    size_t numKeypoints;
    file.read(reinterpret_cast<char*>(&numKeypoints), sizeof(numKeypoints));
    
    // Read each keypoint
    keypoints.resize(numKeypoints);
    for (size_t i = 0; i < numKeypoints; ++i) {
        cv::KeyPoint& kp = keypoints[i];
        file.read(reinterpret_cast<char*>(&kp.pt.x), sizeof(float));
        file.read(reinterpret_cast<char*>(&kp.pt.y), sizeof(float));
        file.read(reinterpret_cast<char*>(&kp.size), sizeof(float));
        file.read(reinterpret_cast<char*>(&kp.angle), sizeof(float));
        file.read(reinterpret_cast<char*>(&kp.response), sizeof(float));
        file.read(reinterpret_cast<char*>(&kp.octave), sizeof(int));
        file.read(reinterpret_cast<char*>(&kp.class_id), sizeof(int));
    }
    
    file.close();
    return true;
}

bool FeatureIO::loadDescriptors(cv::Mat& descriptors, 
                              const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Unable to open file for reading descriptors: " << filePath << std::endl;
        return false;
    }
    
    // Read matrix dimensions and type
    int rows, cols, type;
    
    file.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    file.read(reinterpret_cast<char*>(&cols), sizeof(cols));
    file.read(reinterpret_cast<char*>(&type), sizeof(type));
    
    // Create matrix of correct size and type
    descriptors.create(rows, cols, type);
    
    // Read the matrix data
    file.read(reinterpret_cast<char*>(descriptors.data), 
             descriptors.total() * descriptors.elemSize());
    
    file.close();
    return true;
}

} // namespace ORB_SLAM3