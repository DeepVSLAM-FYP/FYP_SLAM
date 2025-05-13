/**
* This file is part of ORB-SLAM3
*
* SuperPointPipelinedProcess.cc - Implementation of SuperPoint DPU pipelined feature extraction
*/

#include "PipelinedFE/SuperPointPipelinedProcess.h"
#include <iostream>
#include <chrono>
#include <stdexcept>

namespace ORB_SLAM3
{

SuperPointPipelinedProcess::SuperPointPipelinedProcess(
    ThreadSafeQueue<InputQueueItem>& inputQueue, 
    ThreadSafeQueue<ResultQueueItem>& outputQueue,
    const std::string& modelName,
    int numThreads)
    : BasePipelinedProcess(inputQueue, outputQueue),
      mModelName(modelName),
      mNumThreads(numThreads)
{
    // Initialize SuperPointFast
    try {
        mExtractor = std::make_unique<SuperPointFast>(modelName, numThreads);
        std::cout << "SuperPointFast initialized with model: " << modelName << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize SuperPointFast: " << e.what() << std::endl;
        throw;
    }
}

SuperPointPipelinedProcess::~SuperPointPipelinedProcess()
{
    if (mbRunning) {
        Stop();
    }
}

void SuperPointPipelinedProcess::Run()
{
    std::cout << "SuperPointPipelinedProcess: Starting processing thread" << std::endl;
    
    try {
        // Directly use SuperPointFast's run method with the input and output queues
        // This allows SuperPointFast to manage its own internal pipeline and batching
        mExtractor->run(mInputQueue, mOutputQueue);
        
        std::cout << "SuperPointPipelinedProcess: Processing thread completed" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error in SuperPointPipelinedProcess: " << e.what() << std::endl;
    }
}

ResultQueueItem SuperPointPipelinedProcess::ProcessItem(const InputQueueItem& input)
{
    // This function should never be called because SuperPointFast handles the queue processing directly
    throw std::runtime_error("SuperPointPipelinedProcess::ProcessItem should not be called - SuperPointFast processes the queue directly");
}

} // namespace ORB_SLAM3 