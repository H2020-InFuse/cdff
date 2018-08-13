/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesDescription2DExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;

namespace dfn_ci
{

FeaturesDescription2DExecutor::FeaturesDescription2DExecutor(FeaturesDescription2DInterface* dfn)
	{
	this->dfn = dfn;
	}

FeaturesDescription2DExecutor::~FeaturesDescription2DExecutor()
	{

	}

void FeaturesDescription2DExecutor::Execute(FrameConstPtr inputFrame, VisualPointFeatureVector2DConstPtr inputVector, VisualPointFeatureVector2DConstPtr& outputVector)
	{
	Execute(*inputFrame, *inputVector, outputVector);
	}

void FeaturesDescription2DExecutor::Execute(FrameConstPtr inputFrame, VisualPointFeatureVector2DConstPtr inputVector, VisualPointFeatureVector2DPtr outputVector)
	{
	ASSERT(outputVector != NULL, "FeaturesDescription2DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputFrame, *inputVector, *outputVector);
	}

void FeaturesDescription2DExecutor::Execute(const Frame& inputFrame, const VisualPointFeatureVector2D& inputVector, VisualPointFeatureVector2DConstPtr& outputVector)
	{
	ASSERT( outputVector == NULL, "FeaturesDescription2DExecutor, Calling instance creation executor with a non-NULL pointer");
	if (dfn == NULL)
		{
		outputVector = &inputVector;
		return;
		}
	dfn->frameInput(inputFrame);
	dfn->featuresInput(inputVector);
	dfn->process();
	outputVector = & ( dfn->featuresOutput() );
	}

void FeaturesDescription2DExecutor::Execute(const Frame& inputFrame, const VisualPointFeatureVector2D& inputVector, VisualPointFeatureVector2D& outputVector)
	{
	if (dfn == NULL)
		{
		Copy(inputVector, outputVector);
		return;
		}
	dfn->frameInput(inputFrame);
	dfn->featuresInput(inputVector);
	dfn->process();
	Copy( dfn->featuresOutput(), outputVector);
	}
}

/** @} */
