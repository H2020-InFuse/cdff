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

PointCloudAssemblyExecutor::PointCloudAssemblyExecutor(PointCloudAssemblyInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "PointCloudAssemblyExecutor: null dfn in input");
	}

PointCloudAssemblyExecutor::~PointCloudAssemblyExecutor()
	{

	}

void PointCloudAssemblyExecutor::Execute(PointCloudConstPtr inputFirstCloud, PointCloudConstPtr inputSecondCloud, PointCloudConstPtr& outputAssembledCloud)
	{
	Execute(*inputFirstCloud, *inputSecondCloud, outputAssembledCloud);
	}

void PointCloudAssemblyExecutor::Execute(PointCloudConstPtr inputFirstCloud, PointCloudConstPtr inputSecondCloud, PointCloudPtr outputAssembledCloud)
	{
	ASSERT(outputAssembledCloud != NULL, "PointCloudAssemblyExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputFirstCloud, *inputSecondCloud, *outputAssembledCloud);
	}

void PointCloudAssemblyExecutor::Execute(const PointCloud& inputFirstCloud, const PointCloud& inputSecondCloud, PointCloudConstPtr& outputAssembledCloud)
	{
	ASSERT( outputAssembledCloud == NULL, "PointCloudAssemblyExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->firstPointCloudInput(inputFirstCloud);
	dfn->secondPointCloudInput(inputSecondCloud);
	dfn->process();
	outputAssembledCloud = & ( dfn->assembledCloudOutput() );
	}

void PointCloudAssemblyExecutor::Execute(const PointCloud& inputFirstCloud, const PointCloud& inputSecondCloud, PointCloud& outputAssembledCloud)
	{
	dfn->firstPointCloudInput(inputFirstCloud);
	dfn->secondPointCloudInput(inputSecondCloud);
	dfn->process();
	Copy( dfn->assembledCloudOutput(), outputAssembledCloud);
	}

void PointCloudAssemblyExecutor::Execute(PointCloudConstPtr cloud, Pose3DConstPtr viewCenter, float viewRadius, PointCloudConstPtr& outputAssembledCloud)
	{
	Execute(*cloud, *viewCenter, viewRadius, outputAssembledCloud);
	}

void PointCloudAssemblyExecutor::Execute(PointCloudConstPtr cloud, Pose3DConstPtr viewCenter, float viewRadius, PointCloudPtr outputAssembledCloud)
	{
	ASSERT(outputAssembledCloud != NULL, "PointCloudAssemblyExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*cloud, *viewCenter, viewRadius, *outputAssembledCloud);
	}

void PointCloudAssemblyExecutor::Execute(const PointCloud& cloud, const Pose3D& viewCenter, float viewRadius, PointCloudConstPtr& outputAssembledCloud)
	{
	ASSERT( outputAssembledCloud == NULL, "PointCloudAssemblyExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->firstPointCloudInput(cloud);
	dfn->viewCenterInput(viewCenter);
	dfn->viewRadiusInput(viewRadius);
	dfn->process();
	outputAssembledCloud = & ( dfn->assembledCloudOutput() );
	}

void PointCloudAssemblyExecutor::Execute(const PointCloud& cloud, const Pose3D& viewCenter, float viewRadius, PointCloud& outputAssembledCloud)
	{
	dfn->firstPointCloudInput(cloud);
	dfn->viewCenterInput(viewCenter);
	dfn->viewRadiusInput(viewRadius);
	dfn->process();
	Copy( dfn->assembledCloudOutput(), outputAssembledCloud);
	}

}
}

/** @} */
