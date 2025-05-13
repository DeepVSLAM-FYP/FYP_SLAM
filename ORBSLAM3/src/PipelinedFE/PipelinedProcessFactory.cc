/**
* This file is part of ORB-SLAM3
*
* PipelinedProcessFactory.cc - Factory for creating pipelined feature extraction processes
*/

#include "PipelinedFE/PipelinedProcessFactory.h"
#include "PipelinedFE/ORBPipelinedProcess.h"
#include "PipelinedFE/DummyPipelinedProcess.h"

// Include the SuperPoint implementation only if enabled
#ifdef BUILD_SP_DPU
#include "PipelinedFE/SuperPointPipelinedProcess.h"
#endif

namespace ORB_SLAM3
{

std::unique_ptr<BasePipelinedProcess> PipelinedProcessFactory::CreatePipelinedProcess(
    FeatureExtractorType type,
    ThreadSafeQueue<InputQueueItem>& inputQueue,
    ThreadSafeQueue<ResultQueueItem>& outputQueue,
    const std::string& featureDir,
    const std::string& modelName,
    int numThreads)
{
    switch (type)
    {
        case FeatureExtractorType::ORB:
            return std::make_unique<ORBPipelinedProcess>(inputQueue, outputQueue);
        
        case FeatureExtractorType::SUPERPOINT:
            // Use real SuperPoint DPU if enabled, otherwise fallback to dummy
            #ifdef BUILD_SP_DPU
            if (featureDir.empty()) {
                return std::make_unique<SuperPointPipelinedProcess>(
                    inputQueue, 
                    outputQueue,
                    modelName,
                    numThreads
                );
            }
            #endif
            
            // Fallback to dummy process that loads pre-extracted SuperPoint features
            return std::make_unique<DummyPipelinedProcess>(
                inputQueue, 
                outputQueue,
                featureDir,
                "SP",
                CV_32F,
                256  // SuperPoint descriptor size
            );
        
        case FeatureExtractorType::DUMMY:
            // For general dummy process with default settings
            return std::make_unique<DummyPipelinedProcess>(
                inputQueue, 
                outputQueue,
                featureDir,
                "SP",
                CV_32F,
                256  // SuperPoint descriptor size
            );
        
        default:
            // Default to ORB
            return std::make_unique<ORBPipelinedProcess>(inputQueue, outputQueue);
    }
}

} // namespace ORB_SLAM3 