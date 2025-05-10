#ifndef DUMMYPIPELINEDPROCESS_H
#define DUMMYPIPELINEDPROCESS_H

#include "BasePipelinedProcess.h"
#include "../FeatureExtractors/Dummy/FeatureIO.h"
#include <string>

namespace ORB_SLAM3
{

class DummyPipelinedProcess : public BasePipelinedProcess
{
public:
    /**
     * Constructor for the dummy pipelined process
     * @param inputQueue The queue from which to read input items
     * @param outputQueue The queue to which processed results are written
     * @param featuresDir The directory where precomputed features are stored
     * @param extractorType The type of features to load (e.g., "SP" for SuperPoint)
     * @param descriptorType The descriptor type (CV_8U or CV_32F)
     * @param descriptorSize The size of the descriptor (256 for SuperPoint)
     */
    DummyPipelinedProcess(
        ThreadSafeQueue<InputQueueItem>& inputQueue, 
        ThreadSafeQueue<ResultQueueItem>& outputQueue,
        const std::string& featuresDir,
        const std::string& extractorType = "SP",
        int descriptorType = CV_32F,
        int descriptorSize = 256);

    /**
     * Destructor
     */
    ~DummyPipelinedProcess() override;

protected:
    /**
     * Main processing function that runs in a separate thread
     */
    void Run() override;

    /**
     * Process a single input item and return the result
     * @param input The input item to process
     * @return The processed result
     */
    ResultQueueItem ProcessItem(const InputQueueItem& input) override;

private:
    /**
     * Extract the base filename from a full file path
     * @param filepath The full file path
     * @return The base filename without extension
     */
    std::string GetBaseFilename(const std::string& filepath);
    
    /**
     * Load features (keypoints and descriptors) for an image
     * @param imageFilename The image filename
     * @param keypoints Output vector to store loaded keypoints
     * @param descriptors Output matrix to store loaded descriptors
     * @return True if features were loaded successfully
     */
    bool LoadFeatures(const std::string& imageFilename, 
                     std::vector<cv::KeyPoint>& keypoints,
                     cv::Mat& descriptors);
    
    // Directory containing precomputed features
    std::string mFeaturesDir;
    
    // Extractor configuration
    std::string mExtractorType;
    int mDescriptorType;
    int mDescriptorSize;
};

} // namespace ORB_SLAM3

#endif // DUMMYPIPELINEDPROCESS_H 