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

//=====================================================================================================================
DepthFilteringExecutor::DepthFilteringExecutor(DepthFilteringInterface* dfn)
{
    this->dfn = dfn;
    ASSERT(dfn != NULL, "DepthFilteringExecutor: null dfn in input");
}

//=====================================================================================================================
void DepthFilteringExecutor::Execute(FrameWrapper::FrameConstPtr inputFrame, FrameWrapper::FrameConstPtr& outputFrame)
{
    Execute(*inputFrame, outputFrame);
}

//=====================================================================================================================
void DepthFilteringExecutor::Execute(FrameWrapper::FrameConstPtr inputFrame, FrameWrapper::FrameConstPtr outputFrame)
{
    ASSERT(outputFrame != NULL, "DepthFilteringExecutor, Calling NO instance creation Executor with a NULL pointer");
    Execute(*inputFrame, outputFrame);
}

//=====================================================================================================================
void DepthFilteringExecutor::Execute(const FrameWrapper::Frame& inputFrame, FrameWrapper::FrameConstPtr& outputFrame)
{
    ASSERT( outputFrame == NULL, "DepthFilteringExecutor, Calling instance creation executor with a non-NULL pointer");
    dfn->frameInput(inputFrame);
    dfn->process();
    outputFrame = & ( dfn->frameOutput() );
}

//=====================================================================================================================
void DepthFilteringExecutor::Execute(const FrameWrapper::Frame& inputFrame, FrameWrapper::Frame& outputFrame)
{
    dfn->frameInput(inputFrame);
    dfn->process();
    FrameWrapper::Copy( dfn->frameOutput(), outputFrame);
}

}
}

/** @} */
