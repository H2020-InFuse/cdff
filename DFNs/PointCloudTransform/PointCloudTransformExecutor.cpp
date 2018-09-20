/**
 * @addtogroup DFNs
 * @{
 */

#include "PointCloudTransformExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace PointCloudWrapper;
using namespace PoseWrapper;

namespace CDFF
{
namespace DFN
{

PointCloudTransformExecutor::PointCloudTransformExecutor(PointCloudTransformInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "PointCloudTransformExecutor: null dfn in input");
	}

PointCloudTransformExecutor::~PointCloudTransformExecutor()
	{

	}

void PointCloudTransformExecutor::Execute(PointCloudConstPtr inputCloud, Pose3DConstPtr inputPose, PointCloudConstPtr& outputCloud)
	{
	Execute(*inputCloud, *inputPose, outputCloud);
	}

void PointCloudTransformExecutor::Execute(PointCloudConstPtr inputCloud, Pose3DConstPtr inputPose, PointCloudPtr outputCloud)
	{
	ASSERT(outputCloud != NULL, "PointCloudTransformExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputCloud, *inputPose, *outputCloud);
	}

void PointCloudTransformExecutor::Execute(const PointCloud& inputCloud, const Pose3D& inputPose, PointCloudConstPtr& outputCloud)
	{
	ASSERT( outputCloud == NULL, "PointCloudTransformExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->pointCloudInput(inputCloud);
	dfn->poseInput(inputPose);
	dfn->process();
	outputCloud = & ( dfn->transformedPointCloudOutput() );
	}

void PointCloudTransformExecutor::Execute(const PointCloud& inputCloud, const Pose3D& inputPose, PointCloud& outputCloud)
	{
	dfn->pointCloudInput(inputCloud);
	dfn->poseInput(inputPose);
	dfn->process();
	Copy( dfn->transformedPointCloudOutput(), outputCloud);
	}

}
}

/** @} */
