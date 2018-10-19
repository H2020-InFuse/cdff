/**
 * @addtogroup DFNs
 * @{
 */

#include "PointCloudFilteringExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace PointCloudWrapper;
using namespace PoseWrapper;

namespace CDFF
{
namespace DFN
{

PointCloudFilteringExecutor::PointCloudFilteringExecutor(PointCloudFilteringInterface* dfn)
	{
	this->dfn = dfn;
	}

PointCloudFilteringExecutor::~PointCloudFilteringExecutor()
	{

	}

void PointCloudFilteringExecutor::Execute(PointCloudConstPtr inputCloud, PointCloudConstPtr& outputCloud)
	{
	Execute(*inputCloud, outputCloud);
	}

void PointCloudFilteringExecutor::Execute(PointCloudConstPtr inputCloud, PointCloudPtr outputCloud)
	{
	ASSERT(outputCloud != NULL, "PointCloudFilteringExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputCloud, *outputCloud);
	}

void PointCloudFilteringExecutor::Execute(const PointCloud& inputCloud, PointCloudConstPtr& outputCloud)
	{
	ASSERT( outputCloud == NULL, "PointCloudFilteringExecutor, Calling instance creation executor with a non-NULL pointer");
	if (dfn == NULL)
		{
		outputCloud = &inputCloud;
		return;
		}
	dfn->pointCloudInput(inputCloud);
	dfn->process();
	outputCloud = & ( dfn->filteredPointCloudOutput() );
	}

void PointCloudFilteringExecutor::Execute(const PointCloud& inputCloud, PointCloud& outputCloud)
	{
	if (dfn == NULL)
		{
		Copy(inputCloud, outputCloud);
		return;
		}
	dfn->pointCloudInput(inputCloud);
	dfn->process();
	Copy( dfn->filteredPointCloudOutput(), outputCloud);
	}

}
}

/** @} */
