/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREORECONSTRUCTION_EXECUTOR_HPP
#define STEREORECONSTRUCTION_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "StereoReconstructionInterface.hpp"
#include <PointCloud.hpp>
#include <Frame.hpp>

namespace CDFF
{
namespace DFN
{
    class StereoReconstructionExecutor
    {
        public:

            StereoReconstructionExecutor(StereoReconstructionInterface* dfn);
            ~StereoReconstructionExecutor();

	    void Execute(FrameWrapper::FrameConstPtr leftInputFrame, FrameWrapper::FrameConstPtr rightInputFrame, PointCloudWrapper::PointCloudConstPtr& outputCloud);

	    void Execute(FrameWrapper::FrameConstPtr inputFrame, FrameWrapper::FrameConstPtr rightInputFrame, PointCloudWrapper::PointCloudPtr outputCloud);

	    void Execute(const FrameWrapper::Frame& leftInputFrame, const FrameWrapper::Frame& rightInputFrame, PointCloudWrapper::PointCloudConstPtr& outputCloud);

	    void Execute(const FrameWrapper::Frame& leftInputFrame, const FrameWrapper::Frame& rightInputFrame, PointCloudWrapper::PointCloud& outputCloud);

        private:

            StereoReconstructionInterface* dfn;
    };
}
}

#endif // STEREORECONSTRUCTION_EXECUTOR_HPP

/** @} */
