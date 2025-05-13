/**
* This file is part of ORB-SLAM3
*
* PipelinedProcessFactory.h - Factory for creating different pipelined processors
*/

#ifndef PIPELINEDPROCESSFACTORY_H
#define PIPELINEDPROCESSFACTORY_H

#include "PipelinedFE/BasePipelinedProcess.h"
#include "utils/FeatureExtractorTypes.h"
#include <memory>
#include <string>

namespace ORB_SLAM3
{

class PipelinedProcessFactory
{
public:
    /**
     * Creates a pipelined process of the specified type
     * @param type Type of feature extractor to create
     * @param inputQueue Input queue for the process
     * @param outputQueue Output queue for the process
     * @param featureDir Directory containing feature files (for dummy processor)
     * @return A unique pointer to the created processor
     */
    static std::unique_ptr<BasePipelinedProcess> CreatePipelinedProcess(
        FeatureExtractorType type,
        ThreadSafeQueue<InputQueueItem>& inputQueue,
        ThreadSafeQueue<ResultQueueItem>& outputQueue,
        const std::string& featureDir = "/mnt/sda1/FYP_2024/Ruchith/FYP_SLAM/datasets/feature_outputs/SP_H",
        const std::string& modelName = "/root/jupyter_notebooks/Fyp/FYP_SLAM/Thirdparty/super_point_vitis/compiled_SP_by_H.xmodel",
        int numThreads = 4);
};

} // namespace ORB_SLAM3

#endif // PIPELINEDPROCESSFACTORY_H