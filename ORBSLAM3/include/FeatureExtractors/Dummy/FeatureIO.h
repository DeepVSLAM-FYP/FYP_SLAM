/**
 * File: FeatureIO.h
 * Date: April 2025
 * Description: Utilities for saving and loading feature keypoints and descriptors
 * License: see LICENSE.txt
 */

#ifndef FEATURE_IO_H
#define FEATURE_IO_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <string>
#include <vector>
#include <fstream>

namespace ORB_SLAM3 {

/**
 * @brief Class providing utilities to save and load feature keypoints and descriptors
 */
class FeatureIO {
public:
    /**
     * @brief Save keypoints and descriptors to binary files
     * @param keypoints Vector of keypoints to save
     * @param descriptors Matrix of descriptors to save
     * @param keypointsPath Path to save keypoints file
     * @param descriptorsPath Path to save descriptors file
     * @return True if save was successful
     */
    static bool saveFeatures(const std::vector<cv::KeyPoint>& keypoints,
                            const cv::Mat& descriptors,
                            const std::string& keypointsPath,
                            const std::string& descriptorsPath);
    
    /**
     * @brief Save keypoints to a binary file
     * @param keypoints Vector of keypoints to save
     * @param filePath Path to save keypoints file
     * @return True if save was successful
     */
    static bool saveKeypoints(const std::vector<cv::KeyPoint>& keypoints, 
                             const std::string& filePath);
    
    /**
     * @brief Save descriptors to a binary file
     * @param descriptors Matrix of descriptors to save
     * @param filePath Path to save descriptors file
     * @return True if save was successful
     */
    static bool saveDescriptors(const cv::Mat& descriptors, 
                               const std::string& filePath);
    
    /**
     * @brief Load keypoints and descriptors from binary files
     * @param keypoints Output vector to store loaded keypoints
     * @param descriptors Output matrix to store loaded descriptors
     * @param keypointsPath Path to keypoints file
     * @param descriptorsPath Path to descriptors file
     * @return True if load was successful
     */
    static bool loadFeatures(std::vector<cv::KeyPoint>& keypoints,
                            cv::Mat& descriptors,
                            const std::string& keypointsPath,
                            const std::string& descriptorsPath);
    
    /**
     * @brief Load keypoints from a binary file
     * @param keypoints Output vector to store loaded keypoints
     * @param filePath Path to keypoints file
     * @return True if load was successful
     */
    static bool loadKeypoints(std::vector<cv::KeyPoint>& keypoints, 
                             const std::string& filePath);
    
    /**
     * @brief Load descriptors from a binary file
     * @param descriptors Output matrix to store loaded descriptors
     * @param filePath Path to descriptors file
     * @return True if load was successful
     */
    static bool loadDescriptors(cv::Mat& descriptors, 
                              const std::string& filePath);
};

} // namespace ORB_SLAM3

#endif // FEATURE_IO_H