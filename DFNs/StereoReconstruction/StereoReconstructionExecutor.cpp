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

StereoReconstructionExecutor::StereoReconstructionExecutor(StereoReconstructionInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "StereoReconstructionExecutor: null dfn in input");
	}

StereoReconstructionExecutor::~StereoReconstructionExecutor()
	{

	}

void StereoReconstructionExecutor::Execute(FrameConstPtr leftInputFrame, FrameConstPtr rightInputFrame, PointCloudConstPtr& outputCloud)
	{
	Execute(*leftInputFrame, *rightInputFrame, outputCloud);
	}

void StereoReconstructionExecutor::Execute(FrameConstPtr leftInputFrame, FrameConstPtr rightInputFrame, PointCloudPtr outputCloud)
	{
	ASSERT(outputCloud != NULL, "StereoReconstructionExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*leftInputFrame, *rightInputFrame, *outputCloud);
	}

void StereoReconstructionExecutor::Execute(const Frame& leftInputFrame, const Frame& rightInputFrame, PointCloudConstPtr& outputCloud)
	{
	ASSERT( outputCloud == NULL, "StereoReconstructionExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->leftInput(leftInputFrame);
	dfn->rightInput(rightInputFrame);
	dfn->process();
	outputCloud = & ( dfn->pointcloudOutput() );
	}

void StereoReconstructionExecutor::Execute(const Frame& leftInputFrame, const Frame& rightInputFrame, PointCloud& outputCloud)
	{
	dfn->leftInput(leftInputFrame);
	dfn->rightInput(rightInputFrame);
	dfn->process();
	Copy( dfn->pointcloudOutput(), outputCloud);
	}
}
}

/** @} */
