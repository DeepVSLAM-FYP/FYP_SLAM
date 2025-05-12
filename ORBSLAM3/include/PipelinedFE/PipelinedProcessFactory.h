/**
* This file is part of ORB-SLAM3
*
* PipelinedProcessFactory.h - Factory for creating different pipelined processors
*/

#ifndef PIPELINEDPROCESSFACTORY_H
#define PIPELINEDPROCESSFACTORY_H

#include "BasePipelinedProcess.h"
#include "ORBPipelinedProcess.h"
#include "../utils/FeatureExtractorTypes.h"
#include "DummyPipelinedProcess.h"
#include <memory>

namespace ORB_SLAM3
{

class PipelinedProcessFactory
{
public:
    /**
     * Create a pipelined process based on the feature extractor type
     * @param type Type of feature extractor to use
     * @param inputQueue Input queue for the processor
     * @param outputQueue Output queue for the processor
     * @return A unique pointer to the created processor
     */
    static std::unique_ptr<BasePipelinedProcess> CreatePipelinedProcess(
        FeatureExtractorType type,
        ThreadSafeQueue<InputQueueItem>& inputQueue,
        ThreadSafeQueue<ResultQueueItem>& outputQueue,
        const std::string& featureDir = "/mnt/sda1/FYP_2024/Ruchith/FYP_SLAM/datasets/feature_outputs/SP_H")
    {
        switch (type)
        {
            case FeatureExtractorType::ORB:
                return std::make_unique<ORBPipelinedProcess>(inputQueue, outputQueue);
            
            case FeatureExtractorType::SUPERPOINT:
                // Create a dummy process that loads SuperPoint features
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
};

} // namespace ORB_SLAM3

#endif // PIPELINEDPROCESSFACTORY_H