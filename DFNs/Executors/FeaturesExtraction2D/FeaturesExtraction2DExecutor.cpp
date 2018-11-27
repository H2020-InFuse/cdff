/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesExtraction2DExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(FeaturesExtraction2DInterface* dfn, FrameConstPtr inputFrame, VisualPointFeatureVector2DConstPtr& outputVector)
	{
	Execute(dfn, *inputFrame, outputVector);
	}

void Execute(FeaturesExtraction2DInterface* dfn, FrameConstPtr inputFrame, VisualPointFeatureVector2DPtr outputVector)
	{
	ASSERT(outputVector != NULL, "FeaturesExtraction2DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputFrame, *outputVector);
	}

void Execute(FeaturesExtraction2DInterface* dfn, const Frame& inputFrame, VisualPointFeatureVector2DConstPtr& outputVector)
	{
	ASSERT( dfn!= NULL, "FeaturesExtraction2DExecutor, input dfn is null");
	ASSERT( outputVector == NULL, "FeaturesExtraction2DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->frameInput(inputFrame);
	dfn->process();
	outputVector = & ( dfn->featuresOutput() );
	}

void Execute(FeaturesExtraction2DInterface* dfn, const Frame& inputFrame, VisualPointFeatureVector2D& outputVector)
	{
	ASSERT( dfn!= NULL, "FeaturesExtraction2DExecutor, input dfn is null");
	dfn->frameInput(inputFrame);
	dfn->process();
	Copy( dfn->featuresOutput(), outputVector);
	}

}
}
}

/** @} */
