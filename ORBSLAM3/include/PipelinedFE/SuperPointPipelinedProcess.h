/**
* This file is part of ORB-SLAM3
*
* SuperPointPipelinedProcess.h - Class for integrating SuperPoint DPU with pipelined feature extraction
*/

#ifndef SUPERPOINTPIPELINEDPROCESS_H
#define SUPERPOINTPIPELINEDPROCESS_H

#include "PipelinedFE/BasePipelinedProcess.h"
#include "SuperPointFast.h"
#include <memory>
#include <string>

namespace ORB_SLAM3
{

class SuperPointPipelinedProcess : public BasePipelinedProcess
{
public:
    /**
     * Constructor for SuperPointPipelinedProcess
     * @param inputQueue The queue from which to read input items
     * @param outputQueue The queue to which processed results are written
     * @param modelName The name of the SuperPoint DPU model to use
     * @param numThreads The number of processing threads to use
     */
    SuperPointPipelinedProcess(
        ThreadSafeQueue<InputQueueItem>& inputQueue, 
        ThreadSafeQueue<ResultQueueItem>& outputQueue,
        const std::string& modelName,
        int numThreads = 4);
    
    /**
     * Virtual destructor to ensure proper cleanup
     */
    virtual ~SuperPointPipelinedProcess();

protected:
    /**
     * Main processing function that runs in its own thread
     */
    virtual void Run() override;
    
    /**
     * Process a single input item and return the result
     * @param input The input item to process
     * @return The processed result
     * @note This function is not implemented as SuperPointFast handles queue processing directly
     * @see SuperPointPipelinedProcess.cc for implementation details
     */
    virtual ResultQueueItem ProcessItem(const InputQueueItem& input) override;

private:
    // SuperPointFast instance for feature extraction
    std::unique_ptr<SuperPointFast> mExtractor;
    
    // Configuration parameters
    std::string mModelName;
    int mNumThreads;
};

} // namespace ORB_SLAM3

#endif // SUPERPOINTPIPELINEDPROCESS_H 