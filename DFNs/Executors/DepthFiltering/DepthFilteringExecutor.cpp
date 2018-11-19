/**
 * @addtogroup DFNs
 * @{
 */

#include "DepthFilteringExecutor.hpp"
#include <Errors/Assert.hpp>

namespace CDFF
{
namespace DFN
{
namespace Executors
{

//=====================================================================================================================
void Execute(DepthFilteringInterface* dfn, FrameWrapper::FrameConstPtr inputFrame, FrameWrapper::FrameConstPtr& outputFrame)
{
    Execute(dfn, *inputFrame, outputFrame);
}

//=====================================================================================================================
void Execute(DepthFilteringInterface* dfn, FrameWrapper::FrameConstPtr inputFrame, FrameWrapper::FrameConstPtr outputFrame)
{
    ASSERT(outputFrame != NULL, "DepthFilteringExecutor, Calling NO instance creation Executor with a NULL pointer");
    Execute(dfn, *inputFrame, outputFrame);
}

//=====================================================================================================================
void Execute(DepthFilteringInterface* dfn, const FrameWrapper::Frame& inputFrame, FrameWrapper::FrameConstPtr& outputFrame)
{
    ASSERT( dfn!= NULL, "DepthFilteringExecutor, input dfn is null");
    ASSERT( outputFrame == NULL, "DepthFilteringExecutor, Calling instance creation executor with a non-NULL pointer");
    dfn->frameInput(inputFrame);
    dfn->process();
    outputFrame = & ( dfn->frameOutput() );
}

//=====================================================================================================================
void Execute(DepthFilteringInterface* dfn, const FrameWrapper::Frame& inputFrame, FrameWrapper::Frame& outputFrame)
{
    ASSERT( dfn!= NULL, "DepthFilteringExecutor, input dfn is null");
    dfn->frameInput(inputFrame);
    dfn->process();
    FrameWrapper::Copy( dfn->frameOutput(), outputFrame);
}

}
}
}

/** @} */
