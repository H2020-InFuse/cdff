/**
 * @addtogroup DFNs
 * @{
 */

#include "ImageFilteringExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace FrameWrapper;

namespace dfn_ci
{

ImageFilteringExecutor::ImageFilteringExecutor(ImageFilteringInterface* dfn)
	{
	this->dfn = dfn;
	}

ImageFilteringExecutor::~ImageFilteringExecutor()
	{

	}

void ImageFilteringExecutor::Execute(FrameConstPtr inputFrame, FrameConstPtr& outputFrame)
	{
	Execute(*inputFrame, outputFrame);
	}

void ImageFilteringExecutor::Execute(FrameConstPtr inputFrame, FramePtr outputFrame)
	{
	ASSERT(outputFrame != NULL, "ImageFilteringExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputFrame, *outputFrame);
	}

void ImageFilteringExecutor::Execute(const Frame& inputFrame, FrameConstPtr& outputFrame)
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

void ImageFilteringExecutor::Execute(const Frame& inputFrame, Frame& outputFrame)
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

/** @} */
