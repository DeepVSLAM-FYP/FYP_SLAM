#include "PipelinedFE/DummyPipelinedProcess.h"
#include "FeatureExtractors/Dummy/FeatureIO.h"
#include <chrono>
#include <iostream>

namespace ORB_SLAM3
{

DummyPipelinedProcess::DummyPipelinedProcess(
    ThreadSafeQueue<InputQueueItem>& inputQueue, 
    ThreadSafeQueue<ResultQueueItem>& outputQueue,
    const std::string& featuresDir,
    const std::string& extractorType,
    int descriptorType,
    int descriptorSize)
    : BasePipelinedProcess(inputQueue, outputQueue),
      mFeaturesDir(featuresDir),
      mExtractorType(extractorType),
      mDescriptorType(descriptorType),
      mDescriptorSize(descriptorSize)
{
    // Print the features directory for debugging
    std::cout << "Using features from: " << mFeaturesDir << std::endl;
}

DummyPipelinedProcess::~DummyPipelinedProcess()
{
    // No resources to clean up
}

void DummyPipelinedProcess::Run()
{
    InputQueueItem input;
    while (mbRunning && !mInputQueue.is_shutdown())
    {
        if (mInputQueue.try_dequeue_for(input, std::chrono::milliseconds(100)))
        {
            ResultQueueItem result = ProcessItem(input);
            mOutputQueue.enqueue(std::move(result));
        }
    }
    
    // Signal that we're done processing
    mOutputQueue.shutdown();
}

ResultQueueItem DummyPipelinedProcess::ProcessItem(const InputQueueItem& input)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    
    ResultQueueItem result;
    result.index = input.index;
    result.timestamp = input.timestamp;
    result.filename = input.filename;
    result.image = input.image.clone();
    
    // Load features directly using our helper method
    bool success = LoadFeatures(input.filename, result.keypoints, result.descriptors);
    
    if (!success) {
        // If loading failed, return empty keypoints and descriptors
        result.keypoints.clear();
        result.descriptors = cv::Mat();
    }
    else {
        // Validate descriptor dimensions if load was successful
        if (!result.descriptors.empty() && result.descriptors.cols != mDescriptorSize)
        {
            std::cerr << "[DummyPipelinedProcess] Warning: Descriptor dimension mismatch for " 
                    << input.filename << ": got " << result.descriptors.cols 
                    << " but expected " << mDescriptorSize << std::endl;
        }
    }
    
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    
    // Debug output
    // if (input.index % 20 == 0)
    // {
    //     std::cout << "[DummyPipelinedProcess] Processed frame " << input.index 
    //               << " with " << result.keypoints.size() << " keypoints in " 
    //               << ttrack * 1000 << " ms" << std::endl;
    // }
    
    return result;
}

std::string DummyPipelinedProcess::GetBaseFilename(const std::string& filepath)
{
    // Simple string-based implementation to extract the filename without extension
    size_t lastSlash = filepath.find_last_of("/\\");
    std::string filename = (lastSlash == std::string::npos) ? filepath : filepath.substr(lastSlash + 1);
    
    size_t lastDot = filename.find_last_of(".");
    return (lastDot == std::string::npos) ? filename : filename.substr(0, lastDot);
}

bool DummyPipelinedProcess::LoadFeatures(const std::string& imageFilename, 
                                        std::vector<cv::KeyPoint>& keypoints,
                                        cv::Mat& descriptors)
{
    // Extract base filename without directory or extension
    std::string baseFilename = GetBaseFilename(imageFilename);
    
    // Ensure directories have trailing slashes if not empty
    std::string keypointsDir = mFeaturesDir.empty() ? "" : 
                              (mFeaturesDir.back() == '/' ? mFeaturesDir : mFeaturesDir + "/") + "kpts";
    std::string descriptorsDir = mFeaturesDir.empty() ? "" : 
                                (mFeaturesDir.back() == '/' ? mFeaturesDir : mFeaturesDir + "/") + "desc";
    
    // Try loading keypoints with .kp extension first, then .kpts if that fails
    std::string kptsFilePath = keypointsDir + "/" + baseFilename + ".kp";
    bool kptsLoaded = FeatureIO::loadKeypoints(keypoints, kptsFilePath);
    
    if (!kptsLoaded) {
        // Try with .kpts extension
        kptsFilePath = keypointsDir + "/" + baseFilename + ".kpts";
        kptsLoaded = FeatureIO::loadKeypoints(keypoints, kptsFilePath);
        
        if (!kptsLoaded) {
            std::cerr << "Cannot load keypoints file with either .kp or .kpts extension for: " << baseFilename << std::endl;
            return false;
        }
    }
    
    // Load descriptors
    std::string descFilePath = descriptorsDir + "/" + baseFilename + ".desc";
    bool descLoaded = FeatureIO::loadDescriptors(descriptors, descFilePath);
    if (!descLoaded) {
        std::cerr << "Cannot load descriptors file: " << descFilePath << std::endl;
        keypoints.clear();
        return false;
    }
    
    return true;
}

} // namespace ORB_SLAM3 