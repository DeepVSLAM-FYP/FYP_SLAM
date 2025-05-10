#include "PipelinedFE/ORBPipelinedProcess.h"
#include "FeatureExtractors/ORBextractor.h"
#include <chrono>
#include <iostream>

namespace ORB_SLAM3
{

ORBPipelinedProcess::ORBPipelinedProcess(
    ThreadSafeQueue<InputQueueItem>& inputQueue, 
    ThreadSafeQueue<ResultQueueItem>& outputQueue)
    : BasePipelinedProcess(inputQueue, outputQueue)
{
    // Initialize ORB extractor with default parameters
    int nFeatures = 1000;
    float scaleFactor = 1.2f;
    int nLevels = 8;
    int iniThFAST = 20;
    int minThFAST = 7;
    
    mpORBextractor = std::make_unique<ORBextractor>(
        nFeatures, scaleFactor, nLevels, iniThFAST, minThFAST);
}

ORBPipelinedProcess::~ORBPipelinedProcess()
{
    // Smart pointer will handle cleanup
}

void ORBPipelinedProcess::Run()
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

ResultQueueItem ORBPipelinedProcess::ProcessItem(const InputQueueItem& input)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    
    ResultQueueItem result;
    result.index = input.index;
    result.timestamp = input.timestamp;
    result.filename = input.filename;
    result.image = input.image.clone();
    
    // Extract ORB features
    result.lappingArea = {0, 1000};  // Default lapping area
    (*mpORBextractor)(result.image, cv::Mat(), result.keypoints, result.descriptors, result.lappingArea);
    
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    
    // Debug output
    // if (input.index % 20 == 0)
    // {
    //     std::cout << "[ORBPipelinedProcess] Processed frame " << input.index 
    //               << " with " << result.keypoints.size() << " keypoints in " 
    //               << ttrack * 1000 << " ms" << std::endl;
    // }
    
    return result;
}

} // namespace ORB_SLAM3 