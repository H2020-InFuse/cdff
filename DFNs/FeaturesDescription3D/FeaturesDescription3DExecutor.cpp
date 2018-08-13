/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesDescription3DExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;

namespace dfn_ci
{

FeaturesDescription3DExecutor::FeaturesDescription3DExecutor(FeaturesDescription3DInterface* dfn)
	{
	this->dfn = dfn;
	}

FeaturesDescription3DExecutor::~FeaturesDescription3DExecutor()
	{

	}

void FeaturesDescription3DExecutor::Execute(PointCloudConstPtr inputCloud, VisualPointFeatureVector3DConstPtr inputVector, VisualPointFeatureVector3DConstPtr& outputVector)
	{
	Execute(*inputCloud, *inputVector, outputVector);
	}

void FeaturesDescription3DExecutor::Execute(PointCloudConstPtr inputCloud, VisualPointFeatureVector3DConstPtr inputVector, VisualPointFeatureVector3DPtr outputVector)
	{
	ASSERT(outputVector != NULL, "FeaturesDescription3DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputCloud, *inputVector, *outputVector);
	}

void FeaturesDescription3DExecutor::Execute(const PointCloud& inputCloud, const VisualPointFeatureVector3D& inputVector, VisualPointFeatureVector3DConstPtr& outputVector)
	{
	ASSERT( outputVector == NULL, "FeaturesDescription3DExecutor, Calling instance creation executor with a non-NULL pointer");
	if (dfn == NULL)
		{
		outputVector = &inputVector;
		}
	dfn->pointcloudInput(inputCloud);
	dfn->featuresInput(inputVector);
	dfn->process();
	outputVector = & ( dfn->featuresOutput() );
	}

void FeaturesDescription3DExecutor::Execute(const PointCloud& inputCloud, const VisualPointFeatureVector3D& inputVector, VisualPointFeatureVector3D& outputVector)
	{
	if (dfn == NULL)
		{
		Copy(inputVector, outputVector);
		}
	dfn->pointcloudInput(inputCloud);
	dfn->featuresInput(inputVector);
	dfn->process();
	Copy( dfn->featuresOutput(), outputVector);
	}

void FeaturesDescription3DExecutor::Execute(PointCloudConstPtr inputCloud, VisualPointFeatureVector3DConstPtr inputVector, PointCloudConstPtr normalCloud, VisualPointFeatureVector3DConstPtr& outputVector)
	{
	Execute(*inputCloud, *inputVector, *normalCloud, outputVector);
	}

void FeaturesDescription3DExecutor::Execute(PointCloudConstPtr inputCloud, VisualPointFeatureVector3DConstPtr inputVector, PointCloudConstPtr normalCloud, VisualPointFeatureVector3DPtr outputVector)
	{
	ASSERT(outputVector != NULL, "FeaturesDescription3DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputCloud, *inputVector, *normalCloud, *outputVector);
	}

void FeaturesDescription3DExecutor::Execute(const PointCloud& inputCloud, const VisualPointFeatureVector3D& inputVector, const PointCloud& normalCloud, VisualPointFeatureVector3DConstPtr& outputVector)
	{
	ASSERT( outputVector == NULL, "FeaturesDescription3DExecutor, Calling instance creation executor with a non-NULL pointer");
	if (dfn == NULL)
		{
		outputVector = &inputVector;
		return;
		}
	dfn->pointcloudInput(inputCloud);
	dfn->featuresInput(inputVector);
	dfn->normalsInput(normalCloud);
	dfn->process();
	outputVector = & ( dfn->featuresOutput() );
	}

void FeaturesDescription3DExecutor::Execute(const PointCloud& inputCloud, const VisualPointFeatureVector3D& inputVector, const PointCloud& normalCloud, VisualPointFeatureVector3D& outputVector)
	{
	if (dfn == NULL)
		{
		Copy(inputVector, outputVector);
		return;
		}
	dfn->pointcloudInput(inputCloud);
	dfn->featuresInput(inputVector);
	dfn->normalsInput(normalCloud);
	dfn->process();
	Copy( dfn->featuresOutput(), outputVector);
	}
}

/** @} */
