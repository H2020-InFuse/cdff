/**
 * @addtogroup DFNs
 * @{
 */

#include "ImageFilteringExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace FrameWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(ImageFilteringInterface* dfn, FrameConstPtr inputFrame, FrameConstPtr& outputFrame)
	{
	Execute(dfn, *inputFrame, outputFrame);
	}

void Execute(ImageFilteringInterface* dfn, FrameConstPtr inputFrame, FramePtr outputFrame)
	{
	ASSERT(outputFrame != NULL, "ImageFilteringExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputFrame, *outputFrame);
	}

void Execute(ImageFilteringInterface* dfn, const Frame& inputFrame, FrameConstPtr& outputFrame)
	{
	ASSERT( outputFrame == NULL, "ImageFilteringExecutor, Calling instance creation executor with a non-NULL pointer");
	if (dfn == NULL)
		{
		outputFrame = &inputFrame;
		return;
		}
	dfn->imageInput(inputFrame);
	dfn->process();
	outputFrame = & ( dfn->imageOutput() );
	}

void Execute(ImageFilteringInterface* dfn, const Frame& inputFrame, Frame& outputFrame)
	{
	if (dfn == NULL)
		{
		Copy(inputFrame, outputFrame);
		return;
		}
	dfn->imageInput(inputFrame);
	dfn->process();
	Copy( dfn->imageOutput(), outputFrame);
	}

}
}
}

/** @} */
