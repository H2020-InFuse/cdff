/**
 * @addtogroup DFNs
 * @{
 */

#include "Transform3DEstimationExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace CorrespondenceMap3DWrapper;
using namespace PoseWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(Transform3DEstimationInterface* dfn, CorrespondenceMaps3DSequenceConstPtr inputMatches, Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	Execute(dfn, *inputMatches, outputTransforms, success, error);
	}

void Execute(Transform3DEstimationInterface* dfn, CorrespondenceMaps3DSequenceConstPtr inputMatches, Poses3DSequencePtr outputTransforms, bool& success, float& error)
	{
	ASSERT(outputTransforms != NULL, "Transform3DEstimationExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputMatches, *outputTransforms, success, error);
	}

void Execute(Transform3DEstimationInterface* dfn, const CorrespondenceMaps3DSequence& inputMatches, Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	ASSERT( dfn!= NULL, "Transform3DEstimationExecutor, input dfn is null");
	ASSERT( outputTransforms == NULL, "Transform3DEstimationExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->matchesInput(inputMatches);
	dfn->process();
	outputTransforms = & ( dfn->transformsOutput() );
	success = dfn->successOutput();
	error = dfn->errorOutput();
	}

void Execute(Transform3DEstimationInterface* dfn, const CorrespondenceMaps3DSequence& inputMatches, Poses3DSequence& outputTransforms, bool& success, float& error)
	{
	ASSERT( dfn!= NULL, "Transform3DEstimationExecutor, input dfn is null");
	dfn->matchesInput(inputMatches);
	dfn->process();
	Copy( dfn->transformsOutput(), outputTransforms);
	success = dfn->successOutput();
	error = dfn->errorOutput();
	}

}
}
}

/** @} */
