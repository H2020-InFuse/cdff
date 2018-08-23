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

FeaturesExtraction3DExecutor::FeaturesExtraction3DExecutor(FeaturesExtraction3DInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "FeaturesExtraction3DExecutor: null dfn in input");
	}

FeaturesExtraction3DExecutor::~FeaturesExtraction3DExecutor()
	{

	}

void FeaturesExtraction3DExecutor::Execute(PointCloudConstPtr inputCloud, VisualPointFeatureVector3DConstPtr& outputVector)
	{
	Execute(*inputCloud, outputVector);
	}

void FeaturesExtraction3DExecutor::Execute(PointCloudConstPtr inputCloud, VisualPointFeatureVector3DPtr outputVector)
	{
	ASSERT(outputVector != NULL, "FeaturesExtraction3DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputCloud, *outputVector);
	}

void FeaturesExtraction3DExecutor::Execute(const PointCloud& inputCloud, VisualPointFeatureVector3DConstPtr& outputVector)
	{
	ASSERT( outputVector == NULL, "FeaturesExtraction3DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->pointcloudInput(inputCloud);
	dfn->process();
	outputVector = & ( dfn->featuresOutput() );
	}

void FeaturesExtraction3DExecutor::Execute(const PointCloud& inputCloud, VisualPointFeatureVector3D& outputVector)
	{
	dfn->pointcloudInput(inputCloud);
	dfn->process();
	Copy( dfn->featuresOutput(), outputVector);
	}
}
}

/** @} */
