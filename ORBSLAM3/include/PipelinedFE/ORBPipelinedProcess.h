/**
* This file is part of ORB-SLAM3
*
* ORBPipelinedProcess.h - Adapter class for ORBextractor to work with the pipelined process
*/

#ifndef ORBPIPELINEDPROCESS_H
#define ORBPIPELINEDPROCESS_H

#include "BasePipelinedProcess.h"
#include "../FeatureExtractors/ORBextractor.h"
#include <memory>

namespace ORB_SLAM3
{

class ORBPipelinedProcess : public BasePipelinedProcess
{
public:
    /**
     * Constructor
     * @param inputQueue The queue from which to read input items
     * @param outputQueue The queue to which processed results are written
     * @param nFeatures Number of features to extract
     * @param scaleFactor Scale factor for the pyramid
     * @param nLevels Number of pyramid levels
     * @param iniThFAST Initial FAST threshold
     * @param minThFAST Minimum FAST threshold
     */
    ORBPipelinedProcess(
        ThreadSafeQueue<InputQueueItem>& inputQueue, 
        ThreadSafeQueue<ResultQueueItem>& outputQueue,
        int nFeatures = 1000,
        float scaleFactor = 1.2f,
        int nLevels = 8,
        int iniThFAST = 20,
        int minThFAST = 7)
        : BasePipelinedProcess(inputQueue, outputQueue)
    {
        // Initialize ORB extractor
        mpORBextractor = std::make_unique<ORBextractor>(nFeatures, scaleFactor, nLevels, iniThFAST, minThFAST);
    }

protected:
    /**
     * Main processing loop
     */
    void Run() override
    {
        InputQueueItem input;
        
        while (mbRunning)
        {
            // Attempt to get an item from the queue with a timeout
            if (mInputQueue.try_dequeue_for(input, std::chrono::milliseconds(150)))
            {
                // Process the item and push the result to the output queue
                ResultQueueItem result = ProcessItem(input);
                mOutputQueue.enqueue(result);
            }
        }
    }

    /**
     * Process a single input item using ORB extractor
     * @param input The input item to process
     * @return The processed result with extracted features
     */
    ResultQueueItem ProcessItem(const InputQueueItem& input) override
    {
        // Initialize the result with basic information from the input
        ResultQueueItem result;
        result.index = input.index;
        result.timestamp = input.timestamp;
        result.filename = input.filename;
        result.image = input.image.clone();  // Clone the image for the result

        // Extract features using ORB
        // result.lappingArea.clear();  // This is usually empty for monocular
        result.lappingArea = {0,1000};
        (*mpORBextractor)(input.image, cv::Mat(), result.keypoints, result.descriptors, result.lappingArea);

        return result;
    }

private:
    // ORB extractor instance
    std::unique_ptr<ORBextractor> mpORBextractor;
};

} // namespace ORB_SLAM3

#endif // ORBPIPELINEDPROCESS_H