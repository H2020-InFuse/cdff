/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesMatching2DExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace CorrespondenceMap2DWrapper;
using namespace VisualPointFeatureVector2DWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(FeaturesMatching2DInterface* dfn, VisualPointFeatureVector2DConstPtr inputSourceVector, 
	VisualPointFeatureVector2DConstPtr inputSinkVector, CorrespondenceMap2DConstPtr& outputMatches)
	{
	Execute(dfn, *inputSourceVector, *inputSinkVector, outputMatches);
	}

void Execute(FeaturesMatching2DInterface* dfn, VisualPointFeatureVector2DConstPtr inputSourceVector, 
	VisualPointFeatureVector2DConstPtr inputSinkVector, CorrespondenceMap2DPtr outputMatches)
	{
	ASSERT(outputMatches != NULL, "FeaturesMatching2DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputSourceVector, *inputSinkVector, *outputMatches);
	}

void Execute(FeaturesMatching2DInterface* dfn, const VisualPointFeatureVector2D& inputSourceVector, 
	const VisualPointFeatureVector2D& inputSinkVector, CorrespondenceMap2DConstPtr& outputMatches)
	{
	ASSERT( dfn!= NULL, "FeaturesMatching2DExecutor, input dfn is null");
	ASSERT( outputMatches == NULL, "FeaturesMatching2DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->sourceFeaturesInput(inputSourceVector);
	dfn->sinkFeaturesInput(inputSinkVector);
	dfn->process();
	outputMatches = & ( dfn->matchesOutput() );
	}

void Execute(FeaturesMatching2DInterface* dfn, const VisualPointFeatureVector2D& inputSourceVector, 
	const VisualPointFeatureVector2D& inputSinkVector, CorrespondenceMap2D& outputMatches)
	{
	ASSERT( dfn!= NULL, "FeaturesMatching2DExecutor, input dfn is null");
	dfn->sourceFeaturesInput(inputSourceVector);
	dfn->sinkFeaturesInput(inputSinkVector);
	dfn->process();
	Copy( dfn->matchesOutput(), outputMatches);
	}

}
}
}

/** @} */
