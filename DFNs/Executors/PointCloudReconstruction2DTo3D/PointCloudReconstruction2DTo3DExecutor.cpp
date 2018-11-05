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
namespace Executors
{

void Execute(PointCloudReconstruction2DTo3DInterface* dfn, CorrespondenceMap2DConstPtr inputMatches, Pose3DConstPtr inputPose, PointCloudConstPtr& outputCloud)
	{
	Execute(dfn, *inputMatches, *inputPose, outputCloud);
	}

void Execute(PointCloudReconstruction2DTo3DInterface* dfn, CorrespondenceMap2DConstPtr inputMatches, Pose3DConstPtr inputPose, PointCloudPtr outputCloud)
	{
	ASSERT(outputCloud != NULL, "PointCloudReconstruction2DTo3DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputMatches, *inputPose, *outputCloud);
	}

void Execute(PointCloudReconstruction2DTo3DInterface* dfn, const CorrespondenceMap2D& inputMatches, const Pose3D& inputPose, PointCloudConstPtr& outputCloud)
	{
	ASSERT( dfn!= NULL, "PointCloudReconstruction2DTo3DExecutor, input dfn is null");
	ASSERT( outputCloud == NULL, "PointCloudReconstruction2DTo3DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->matchesInput(inputMatches);
	dfn->poseInput(inputPose);
	dfn->process();
	outputCloud = & ( dfn->pointcloudOutput() );
	}

void Execute(PointCloudReconstruction2DTo3DInterface* dfn, const CorrespondenceMap2D& inputMatches, const Pose3D& inputPose, PointCloud& outputCloud)
	{
	ASSERT( dfn!= NULL, "PointCloudReconstruction2DTo3DExecutor, input dfn is null");
	dfn->matchesInput(inputMatches);
	dfn->poseInput(inputPose);
	dfn->process();
	Copy( dfn->pointcloudOutput(), outputCloud);
	}

}
}
}

/** @} */
