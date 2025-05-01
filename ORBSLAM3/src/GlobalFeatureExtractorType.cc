#include "GlobalFeatureExtractorType.h"

namespace ORB_SLAM3 {

std::string GlobalFeatureExtractorInfo::GetFeatureExtractorType() {
    GlobalFeatureExtractorInfo& instance = GetInstance();
    std::lock_guard<std::mutex> lock(instance.mutex_);
    return instance.featureExtractorType_;
}

void GlobalFeatureExtractorInfo::SetFeatureExtractorType(const std::string& type) {
    GlobalFeatureExtractorInfo& instance = GetInstance();
    std::lock_guard<std::mutex> lock(instance.mutex_);
    instance.featureExtractorType_ = type;
}

float GlobalFeatureExtractorInfo::GetTH_HIGH() {
    GlobalFeatureExtractorInfo& instance = GetInstance();
    std::lock_guard<std::mutex> lock(instance.mutex_);
    return instance.th_high_;
}

float GlobalFeatureExtractorInfo::GetTH_LOW() {
    GlobalFeatureExtractorInfo& instance = GetInstance();
    std::lock_guard<std::mutex> lock(instance.mutex_);
    return instance.th_low_;
}

void GlobalFeatureExtractorInfo::SetTH_HIGH(float th_high) {
    GlobalFeatureExtractorInfo& instance = GetInstance();
    std::lock_guard<std::mutex> lock(instance.mutex_);
    instance.th_high_ = th_high;
}

void GlobalFeatureExtractorInfo::SetTH_LOW(float th_low) {
    GlobalFeatureExtractorInfo& instance = GetInstance();
    std::lock_guard<std::mutex> lock(instance.mutex_);
    instance.th_low_ = th_low;
}

GlobalFeatureExtractorInfo& GlobalFeatureExtractorInfo::GetInstance() {
    static GlobalFeatureExtractorInfo instance;
    return instance;
}

} // namespace ORB_SLAM3