/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoReconstructionExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace FrameWrapper;
using namespace PointCloudWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(StereoReconstructionInterface* dfn, FrameConstPtr leftInputFrame, FrameConstPtr rightInputFrame, PointCloudConstPtr& outputCloud)
	{
	Execute(dfn, *leftInputFrame, *rightInputFrame, outputCloud);
	}

void Execute(StereoReconstructionInterface* dfn, FrameConstPtr leftInputFrame, FrameConstPtr rightInputFrame, PointCloudPtr outputCloud)
	{
	ASSERT(outputCloud != NULL, "StereoReconstructionExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *leftInputFrame, *rightInputFrame, *outputCloud);
	}

void Execute(StereoReconstructionInterface* dfn, const Frame& leftInputFrame, const Frame& rightInputFrame, PointCloudConstPtr& outputCloud)
	{
	ASSERT( dfn!= NULL, "StereoReconstructionExecutor, input dfn is null");
	ASSERT( outputCloud == NULL, "StereoReconstructionExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->leftInput(leftInputFrame);
	dfn->rightInput(rightInputFrame);
	dfn->process();
	outputCloud = & ( dfn->pointcloudOutput() );
	}

void Execute(StereoReconstructionInterface* dfn, const Frame& leftInputFrame, const Frame& rightInputFrame, PointCloud& outputCloud)
	{
	ASSERT( dfn!= NULL, "StereoReconstructionExecutor, input dfn is null");
	dfn->leftInput(leftInputFrame);
	dfn->rightInput(rightInputFrame);
	dfn->process();
	Copy( dfn->pointcloudOutput(), outputCloud);
	}
}
}
}

/** @} */
