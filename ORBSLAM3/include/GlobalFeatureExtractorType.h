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

private:
    // The singleton instance
    static GlobalFeatureExtractorInfo& GetInstance();
    
    // Private constructor for singleton
    GlobalFeatureExtractorInfo() = default;
    
    // The actual feature extractor type
    std::string featureExtractorType_ = "ORB";
    
    // Mutex for thread safety
    std::mutex mutex_;
};

} // namespace ORB_SLAM3