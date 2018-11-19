/**
 * @addtogroup DFNs
 * @{
 */

#include "PointCloudAssemblyExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace PointCloudWrapper;
using namespace PoseWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(PointCloudAssemblyInterface* dfn, PointCloudConstPtr inputFirstCloud, PointCloudConstPtr inputSecondCloud, PointCloudConstPtr& outputAssembledCloud)
	{
	Execute(dfn, *inputFirstCloud, *inputSecondCloud, outputAssembledCloud);
	}

void Execute(PointCloudAssemblyInterface* dfn, PointCloudConstPtr inputFirstCloud, PointCloudConstPtr inputSecondCloud, PointCloudPtr outputAssembledCloud)
	{
	ASSERT(outputAssembledCloud != NULL, "PointCloudAssemblyExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputFirstCloud, *inputSecondCloud, *outputAssembledCloud);
	}

void Execute(PointCloudAssemblyInterface* dfn, const PointCloud& inputFirstCloud, const PointCloud& inputSecondCloud, PointCloudConstPtr& outputAssembledCloud)
	{
	ASSERT( dfn!= NULL, "PointCloudAssemblyExecutor, input dfn is null");
	ASSERT( outputAssembledCloud == NULL, "PointCloudAssemblyExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->firstPointCloudInput(inputFirstCloud);
	dfn->secondPointCloudInput(inputSecondCloud);
	dfn->process();
	outputAssembledCloud = & ( dfn->assembledCloudOutput() );
	}

void Execute(PointCloudAssemblyInterface* dfn, const PointCloud& inputFirstCloud, const PointCloud& inputSecondCloud, PointCloud& outputAssembledCloud)
	{
	ASSERT( dfn!= NULL, "PointCloudAssemblyExecutor, input dfn is null");
	dfn->firstPointCloudInput(inputFirstCloud);
	dfn->secondPointCloudInput(inputSecondCloud);
	dfn->process();
	Copy( dfn->assembledCloudOutput(), outputAssembledCloud);
	}

void Execute(PointCloudAssemblyInterface* dfn, PointCloudConstPtr cloud, Pose3DConstPtr viewCenter, float viewRadius, PointCloudConstPtr& outputAssembledCloud)
	{
	Execute(dfn, *cloud, *viewCenter, viewRadius, outputAssembledCloud);
	}

void Execute(PointCloudAssemblyInterface* dfn, PointCloudConstPtr cloud, Pose3DConstPtr viewCenter, float viewRadius, PointCloudPtr outputAssembledCloud)
	{
	ASSERT(outputAssembledCloud != NULL, "PointCloudAssemblyExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *cloud, *viewCenter, viewRadius, *outputAssembledCloud);
	}

void Execute(PointCloudAssemblyInterface* dfn, const PointCloud& cloud, const Pose3D& viewCenter, float viewRadius, PointCloudConstPtr& outputAssembledCloud)
	{
	ASSERT( dfn!= NULL, "PointCloudAssemblyExecutor, input dfn is null");
	ASSERT( outputAssembledCloud == NULL, "PointCloudAssemblyExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->firstPointCloudInput(cloud);
	dfn->viewCenterInput(viewCenter);
	dfn->viewRadiusInput(viewRadius);
	dfn->process();
	outputAssembledCloud = & ( dfn->assembledCloudOutput() );
	}

void Execute(PointCloudAssemblyInterface* dfn, const PointCloud& cloud, const Pose3D& viewCenter, float viewRadius, PointCloud& outputAssembledCloud)
	{
	ASSERT( dfn!= NULL, "PointCloudAssemblyExecutor, input dfn is null");
	dfn->firstPointCloudInput(cloud);
	dfn->viewCenterInput(viewCenter);
	dfn->viewRadiusInput(viewRadius);
	dfn->process();
	Copy( dfn->assembledCloudOutput(), outputAssembledCloud);
	}

}
}
}

/** @} */
