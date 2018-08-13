/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesMatching3DExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;

namespace dfn_ci
{

FeaturesMatching3DExecutor::FeaturesMatching3DExecutor(FeaturesMatching3DInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "FeaturesMatching3DExecutor: null dfn in input");
	}

FeaturesMatching3DExecutor::~FeaturesMatching3DExecutor()
	{

	}

void FeaturesMatching3DExecutor::Execute(VisualPointFeatureVector3DConstPtr inputSourceVector, VisualPointFeatureVector3DConstPtr inputSinkVector, Pose3DConstPtr& outputTransform, bool& success)
	{
	Execute(*inputSourceVector, *inputSinkVector, outputTransform, success);
	}

void FeaturesMatching3DExecutor::Execute(VisualPointFeatureVector3DConstPtr inputSourceVector, VisualPointFeatureVector3DConstPtr inputSinkVector, Pose3DPtr outputTransform, bool& success)
	{
	ASSERT(outputTransform != NULL, "FeaturesMatching3DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputSourceVector, *inputSinkVector, *outputTransform, success);
	}

void FeaturesMatching3DExecutor::Execute(const VisualPointFeatureVector3D& inputSourceVector, const VisualPointFeatureVector3D& inputSinkVector, Pose3DConstPtr& outputTransform, bool& success)
	{
	ASSERT( outputTransform == NULL, "FeaturesMatching3DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->sourceFeaturesInput(inputSourceVector);
	dfn->sinkFeaturesInput(inputSinkVector);
	dfn->process();
	outputTransform = & ( dfn->transformOutput() );
	success = dfn->successOutput();
	}

void FeaturesMatching3DExecutor::Execute(const VisualPointFeatureVector3D& inputSourceVector, const VisualPointFeatureVector3D& inputSinkVector, Pose3D& outputTransform, bool& success)
	{
	dfn->sourceFeaturesInput(inputSourceVector);
	dfn->sinkFeaturesInput(inputSinkVector);
	dfn->process();
	Copy( dfn->transformOutput(), outputTransform);
	success = dfn->successOutput();
	}
}

/** @} */
