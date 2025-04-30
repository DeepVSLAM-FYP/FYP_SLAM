#include "GlobalFeatureExtractorType.h"

namespace ORB_SLAM3 {

std::string GlobalFeatureExtractorInfo::GetFeatureExtractorType() {
    std::lock_guard<std::mutex> lock(GetInstance().mutex_);
    return GetInstance().featureExtractorType_;
}

void GlobalFeatureExtractorInfo::SetFeatureExtractorType(const std::string& type) {
    std::lock_guard<std::mutex> lock(GetInstance().mutex_);
    GetInstance().featureExtractorType_ = type;
}

GlobalFeatureExtractorInfo& GlobalFeatureExtractorInfo::GetInstance() {
    static GlobalFeatureExtractorInfo instance;
    return instance;
}

} // namespace ORB_SLAM3