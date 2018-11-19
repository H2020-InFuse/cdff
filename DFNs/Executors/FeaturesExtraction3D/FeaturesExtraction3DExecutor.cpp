/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesExtraction3DExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(FeaturesExtraction3DInterface* dfn, PointCloudConstPtr inputCloud, VisualPointFeatureVector3DConstPtr& outputVector)
	{
	Execute(dfn, *inputCloud, outputVector);
	}

void Execute(FeaturesExtraction3DInterface* dfn, PointCloudConstPtr inputCloud, VisualPointFeatureVector3DPtr outputVector)
	{
	ASSERT(outputVector != NULL, "FeaturesExtraction3DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputCloud, *outputVector);
	}

void Execute(FeaturesExtraction3DInterface* dfn, const PointCloud& inputCloud, VisualPointFeatureVector3DConstPtr& outputVector)
	{
	ASSERT( dfn!= NULL, "FeaturesExtraction3DExecutor, input dfn is null");
	ASSERT( outputVector == NULL, "FeaturesExtraction3DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->pointcloudInput(inputCloud);
	dfn->process();
	outputVector = & ( dfn->featuresOutput() );
	}

void Execute(FeaturesExtraction3DInterface* dfn, const PointCloud& inputCloud, VisualPointFeatureVector3D& outputVector)
	{
	ASSERT( dfn!= NULL, "FeaturesExtraction3DExecutor, input dfn is null");
	dfn->pointcloudInput(inputCloud);
	dfn->process();
	Copy( dfn->featuresOutput(), outputVector);
	}

}
}
}

/** @} */
