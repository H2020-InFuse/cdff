/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEFILTERING_EXECUTOR_HPP
#define IMAGEFILTERING_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "ImageFilteringInterface.hpp"
#include <Frame.hpp>

namespace dfn_ci
{
    class ImageFilteringExecutor
    {
        public:

            ImageFilteringExecutor(ImageFilteringInterface* dfn);
            ~ImageFilteringExecutor();

	    void Execute(FrameWrapper::FrameConstPtr inputFrame, FrameWrapper::FrameConstPtr& outputFrame);

	    void Execute(FrameWrapper::FrameConstPtr inputFrame, FrameWrapper::FramePtr outputFrame);

	    void Execute(const FrameWrapper::Frame& inputFrame, FrameWrapper::FrameConstPtr& outputFrame);

	    void Execute(const FrameWrapper::Frame& inputFrame, FrameWrapper::Frame& outputFrame);

        private:

            ImageFilteringInterface* dfn;
    };
}

#endif // IMAGEFILTERING_EXECUTOR_HPP

/** @} */
