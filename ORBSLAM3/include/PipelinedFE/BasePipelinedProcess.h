/**
* This file is part of ORB-SLAM3
*
* BasePipelinedProcess.h - Base class for pipelined feature extraction processes
*/

#ifndef BASEPIPELINEDPROCESS_H
#define BASEPIPELINEDPROCESS_H

#include <thread>
#include <atomic>
#include <memory>
#include <opencv2/core/core.hpp>
#include "utils/ThreadSafeQueue.h"
#include "../utils/FeatureExtractorTypes.h"

namespace ORB_SLAM3
{

// Base class for pipelined processors
class BasePipelinedProcess
{
public:
    /**
     * Constructor for the base pipelined process
     * @param inputQueue The queue from which to read input items
     * @param outputQueue The queue to which processed results are written
     */
    BasePipelinedProcess(
        ThreadSafeQueue<InputQueueItem>& inputQueue, 
        ThreadSafeQueue<ResultQueueItem>& outputQueue)
        : mInputQueue(inputQueue), mOutputQueue(outputQueue), mbRunning(false)
    {}

    /**
     * Virtual destructor to ensure proper cleanup in derived classes
     */
    virtual ~BasePipelinedProcess()
    {
        if (mbRunning)
        {
            Stop();
        }
    }

    /**
     * Start the processing thread
     */
    void StartProcessing()
    {
        if (!mbRunning)
        {
            mbRunning = true;
            mProcessThread = std::thread(&BasePipelinedProcess::Run, this);
        }
    }

    /**
     * Stop the processing thread
     */
    void Stop()
    {
        if (mbRunning)
        {
            mbRunning = false;
            if (mProcessThread.joinable())
            {
                mProcessThread.join();
            }
        }
    }

protected:
    /**
     * Main processing function to be implemented by derived classes
     * This function will run in its own thread and process items from the input queue
     */
    virtual void Run() = 0;

    /**
     * Process a single input item and return the result
     * @param input The input item to process
     * @return The processed result
     */
    virtual ResultQueueItem ProcessItem(const InputQueueItem& input) = 0;

    // Input and output queues
    ThreadSafeQueue<InputQueueItem>& mInputQueue;
    ThreadSafeQueue<ResultQueueItem>& mOutputQueue;

    // Thread control
    std::thread mProcessThread;
    std::atomic<bool> mbRunning;
};

} // namespace ORB_SLAM3

#endif // BASEPIPELINEDPROCESS_H