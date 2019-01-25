/**
 * @addtogroup DFNs
 * @{
 */

#include "PointCloudTransformationExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace PointCloudWrapper;
using namespace PoseWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(PointCloudTransformationInterface* dfn, PointCloudConstPtr inputCloud, Pose3DConstPtr inputPose, PointCloudConstPtr& outputCloud)
	{
	Execute(dfn, *inputCloud, *inputPose, outputCloud);
	}

void Execute(PointCloudTransformationInterface* dfn, PointCloudConstPtr inputCloud, Pose3DConstPtr inputPose, PointCloudPtr outputCloud)
	{
	ASSERT(outputCloud != NULL, "PointCloudTransformationExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputCloud, *inputPose, *outputCloud);
	}

void Execute(PointCloudTransformationInterface* dfn, const PointCloud& inputCloud, const Pose3D& inputPose, PointCloudConstPtr& outputCloud)
	{
	ASSERT( dfn!= NULL, "PointCloudTransformationExecutor, input dfn is null");
	ASSERT( outputCloud == NULL, "PointCloudTransformationExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->pointCloudInput(inputCloud);
	dfn->poseInput(inputPose);
	dfn->process();
	outputCloud = & ( dfn->transformedPointCloudOutput() );
	}

void Execute(PointCloudTransformationInterface* dfn, const PointCloud& inputCloud, const Pose3D& inputPose, PointCloud& outputCloud)
	{
	ASSERT( dfn!= NULL, "PointCloudTransformationExecutor, input dfn is null");
	dfn->poseInput(inputPose);
	dfn->process();
	Copy( dfn->transformedPointCloudOutput(), outputCloud);
	}

}
}
}

/** @} */
