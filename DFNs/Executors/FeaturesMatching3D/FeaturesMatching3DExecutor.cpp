/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesMatching3DExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(FeaturesMatching3DInterface* dfn, VisualPointFeatureVector3DConstPtr inputSourceVector, 
	VisualPointFeatureVector3DConstPtr inputSinkVector, Pose3DConstPtr& outputTransform, bool& success)
	{
	Execute(dfn, *inputSourceVector, *inputSinkVector, outputTransform, success);
	}

void Execute(FeaturesMatching3DInterface* dfn, VisualPointFeatureVector3DConstPtr inputSourceVector, 
	VisualPointFeatureVector3DConstPtr inputSinkVector, Pose3DPtr outputTransform, bool& success)
	{
	ASSERT(outputTransform != NULL, "FeaturesMatching3DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputSourceVector, *inputSinkVector, *outputTransform, success);
	}

void Execute(FeaturesMatching3DInterface* dfn, const VisualPointFeatureVector3D& inputSourceVector, 
	const VisualPointFeatureVector3D& inputSinkVector, Pose3DConstPtr& outputTransform, bool& success)
	{
	ASSERT( dfn!= NULL, "FeaturesMatching3DExecutor, input dfn is null");
	ASSERT( outputTransform == NULL, "FeaturesMatching3DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->sourceFeaturesInput(inputSourceVector);
	dfn->sinkFeaturesInput(inputSinkVector);
	dfn->process();
	outputTransform = & ( dfn->transformOutput() );
	success = dfn->successOutput();
	}

void Execute(FeaturesMatching3DInterface* dfn, const VisualPointFeatureVector3D& inputSourceVector, 
	const VisualPointFeatureVector3D& inputSinkVector, Pose3D& outputTransform, bool& success)
	{
	ASSERT( dfn!= NULL, "FeaturesMatching3DExecutor, input dfn is null");
	dfn->sourceFeaturesInput(inputSourceVector);
	dfn->sinkFeaturesInput(inputSinkVector);
	dfn->process();
	Copy( dfn->transformOutput(), outputTransform);
	success = dfn->successOutput();
	}

}
}
}

/** @} */
