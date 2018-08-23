/**
 * @addtogroup DFNs
 * @{
 */

#include "PointCloudReconstruction2DTo3DExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

namespace CDFF
{
namespace DFN
{

PointCloudReconstruction2DTo3DExecutor::PointCloudReconstruction2DTo3DExecutor(PointCloudReconstruction2DTo3DInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "PointCloudReconstruction2DTo3DExecutor: null dfn in input");
	}

PointCloudReconstruction2DTo3DExecutor::~PointCloudReconstruction2DTo3DExecutor()
	{

	}

void PointCloudReconstruction2DTo3DExecutor::Execute(CorrespondenceMap2DConstPtr inputMatches, Pose3DConstPtr inputPose, PointCloudConstPtr& outputCloud)
	{
	Execute(*inputMatches, *inputPose, outputCloud);
	}

void PointCloudReconstruction2DTo3DExecutor::Execute(CorrespondenceMap2DConstPtr inputMatches, Pose3DConstPtr inputPose, PointCloudPtr outputCloud)
	{
	ASSERT(outputCloud != NULL, "PointCloudReconstruction2DTo3DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputMatches, *inputPose, *outputCloud);
	}

void PointCloudReconstruction2DTo3DExecutor::Execute(const CorrespondenceMap2D& inputMatches, const Pose3D& inputPose, PointCloudConstPtr& outputCloud)
	{
	ASSERT( outputCloud == NULL, "PointCloudReconstruction2DTo3DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->matchesInput(inputMatches);
	dfn->poseInput(inputPose);
	dfn->process();
	outputCloud = & ( dfn->pointcloudOutput() );
	}

void PointCloudReconstruction2DTo3DExecutor::Execute(const CorrespondenceMap2D& inputMatches, const Pose3D& inputPose, PointCloud& outputCloud)
	{
	dfn->matchesInput(inputMatches);
	dfn->poseInput(inputPose);
	dfn->process();
	Copy( dfn->pointcloudOutput(), outputCloud);
	}
}
}

/** @} */
