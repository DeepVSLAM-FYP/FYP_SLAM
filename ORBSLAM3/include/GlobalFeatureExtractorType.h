#pragma once

#include <string>
#include <mutex>

namespace ORB_SLAM3 {

// This class provides a global way to access the feature extractor type
// that is currently being used in the system
class GlobalFeatureExtractorInfo {
public:
    // Get the current feature extractor type (thread-safe)
    static std::string GetFeatureExtractorType();
    
    // Set the current feature extractor type (thread-safe)
    static void SetFeatureExtractorType(const std::string& type);
    
    // Get and set the threshold values (thread-safe)
    static float GetTH_HIGH();
    static float GetTH_LOW();
    static void SetTH_HIGH(float th_high);
    static void SetTH_LOW(float th_low);

private:
    // The singleton instance
    static GlobalFeatureExtractorInfo& GetInstance();
    
    // Private constructor for singleton
    GlobalFeatureExtractorInfo() = default;
    
    // The actual feature extractor type
    std::string featureExtractorType_ = "ORB";
    
    // Descriptor matching threshold values
    float th_high_ = 100;  // Default value matching original code
    float th_low_ = 50;    // Default value matching original code
    
    // Mutex for thread safety
    std::mutex mutex_;
};

} // namespace ORB_SLAM3