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

FeaturesExtraction2DExecutor::FeaturesExtraction2DExecutor(FeaturesExtraction2DInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "FeaturesExtraction2DExecutor: null dfn in input");
	}

FeaturesExtraction2DExecutor::~FeaturesExtraction2DExecutor()
	{

	}

void FeaturesExtraction2DExecutor::Execute(FrameConstPtr inputFrame, VisualPointFeatureVector2DConstPtr& outputVector)
	{
	Execute(*inputFrame, outputVector);
	}

void FeaturesExtraction2DExecutor::Execute(FrameConstPtr inputFrame, VisualPointFeatureVector2DPtr outputVector)
	{
	ASSERT(outputVector != NULL, "FeaturesExtraction2DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputFrame, *outputVector);
	}

void FeaturesExtraction2DExecutor::Execute(const Frame& inputFrame, VisualPointFeatureVector2DConstPtr& outputVector)
	{
	ASSERT( outputVector == NULL, "FeaturesExtraction2DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->frameInput(inputFrame);
	dfn->process();
	outputVector = & ( dfn->featuresOutput() );
	}

void FeaturesExtraction2DExecutor::Execute(const Frame& inputFrame, VisualPointFeatureVector2D& outputVector)
	{
	dfn->frameInput(inputFrame);
	dfn->process();
	Copy( dfn->featuresOutput(), outputVector);
	}
}
}

/** @} */
