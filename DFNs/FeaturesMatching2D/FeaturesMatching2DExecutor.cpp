/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesMatching2DExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace CorrespondenceMap2DWrapper;
using namespace VisualPointFeatureVector2DWrapper;

namespace dfn_ci
{

FeaturesMatching2DExecutor::FeaturesMatching2DExecutor(FeaturesMatching2DInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "FeaturesMatching2DExecutor: null dfn in input");
	}

FeaturesMatching2DExecutor::~FeaturesMatching2DExecutor()
	{

	}

void FeaturesMatching2DExecutor::Execute(VisualPointFeatureVector2DConstPtr inputSourceVector, VisualPointFeatureVector2DConstPtr inputSinkVector, CorrespondenceMap2DConstPtr& outputMatches)
	{
	Execute(*inputSourceVector, *inputSinkVector, outputMatches);
	}

void FeaturesMatching2DExecutor::Execute(VisualPointFeatureVector2DConstPtr inputSourceVector, VisualPointFeatureVector2DConstPtr inputSinkVector, CorrespondenceMap2DPtr outputMatches)
	{
	ASSERT(outputMatches != NULL, "FeaturesMatching2DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputSourceVector, *inputSinkVector, *outputMatches);
	}

void FeaturesMatching2DExecutor::Execute(const VisualPointFeatureVector2D& inputSourceVector, const VisualPointFeatureVector2D& inputSinkVector, CorrespondenceMap2DConstPtr& outputMatches)
	{
	ASSERT( outputMatches == NULL, "FeaturesMatching2DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->sourceFeaturesInput(inputSourceVector);
	dfn->sinkFeaturesInput(inputSinkVector);
	dfn->process();
	outputMatches = & ( dfn->matchesOutput() );
	}

void FeaturesMatching2DExecutor::Execute(const VisualPointFeatureVector2D& inputSourceVector, const VisualPointFeatureVector2D& inputSinkVector, CorrespondenceMap2D& outputMatches)
	{
	dfn->sourceFeaturesInput(inputSourceVector);
	dfn->sinkFeaturesInput(inputSinkVector);
	dfn->process();
	Copy( dfn->matchesOutput(), outputMatches);
	}
}

/** @} */
