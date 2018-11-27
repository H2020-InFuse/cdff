/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesDescription2DExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(FeaturesDescription2DInterface* dfn, FrameConstPtr inputFrame, VisualPointFeatureVector2DConstPtr inputVector, VisualPointFeatureVector2DConstPtr& outputVector)
	{
	Execute(dfn, *inputFrame, *inputVector, outputVector);
	}

void Execute(FeaturesDescription2DInterface* dfn, FrameConstPtr inputFrame, VisualPointFeatureVector2DConstPtr inputVector, VisualPointFeatureVector2DPtr outputVector)
	{
	ASSERT(outputVector != NULL, "FeaturesDescription2DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputFrame, *inputVector, *outputVector);
	}

void Execute(FeaturesDescription2DInterface* dfn, const Frame& inputFrame, const VisualPointFeatureVector2D& inputVector, VisualPointFeatureVector2DConstPtr& outputVector)
	{
	ASSERT( dfn!= NULL, "FeaturesDescription2DExecutor, input dfn is null");
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

void Execute(FeaturesDescription2DInterface* dfn, const Frame& inputFrame, const VisualPointFeatureVector2D& inputVector, VisualPointFeatureVector2D& outputVector)
	{
	ASSERT( dfn!= NULL, "FeaturesDescription2DExecutor, input dfn is null");
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
}
}

/** @} */
