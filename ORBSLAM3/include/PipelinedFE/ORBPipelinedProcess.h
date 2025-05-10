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

class ORBextractor;

class ORBPipelinedProcess : public BasePipelinedProcess
{
public:
    /**
     * Constructor for the ORB pipelined process
     * @param inputQueue The queue from which to read input items
     * @param outputQueue The queue to which processed results are written
     */
    ORBPipelinedProcess(
        ThreadSafeQueue<InputQueueItem>& inputQueue, 
        ThreadSafeQueue<ResultQueueItem>& outputQueue);

    /**
     * Destructor
     */
    ~ORBPipelinedProcess() override;

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
    // The ORB extractor instance
    std::unique_ptr<ORBextractor> mpORBextractor;
};

} // namespace ORB_SLAM3

#endif // ORBPIPELINEDPROCESS_H