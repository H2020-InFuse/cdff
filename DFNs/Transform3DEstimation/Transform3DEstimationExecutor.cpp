/**
 * @addtogroup DFNs
 * @{
 */

#include "Transform3DEstimationExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace CorrespondenceMap3DWrapper;
using namespace PoseWrapper;

namespace dfn_ci
{

Transform3DEstimationExecutor::Transform3DEstimationExecutor(Transform3DEstimationInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "Transform3DEstimationExecutor: null dfn in input");
	}

Transform3DEstimationExecutor::~Transform3DEstimationExecutor()
	{

	}

void Transform3DEstimationExecutor::Execute(CorrespondenceMaps3DSequenceConstPtr inputMatches, Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	Execute(*inputMatches, outputTransforms, success, error);
	}

void Transform3DEstimationExecutor::Execute(CorrespondenceMaps3DSequenceConstPtr inputMatches, Poses3DSequencePtr outputTransforms, bool& success, float& error)
	{
	ASSERT(outputTransforms != NULL, "Transform3DEstimationExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputMatches, *outputTransforms, success, error);
	}

void Transform3DEstimationExecutor::Execute(const CorrespondenceMaps3DSequence& inputMatches, Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	ASSERT( outputTransforms == NULL, "Transform3DEstimationExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->matchesInput(inputMatches);
	dfn->process();
	outputTransforms = & ( dfn->transformsOutput() );
	success = dfn->successOutput();
	error = dfn->errorOutput();
	}

void Transform3DEstimationExecutor::Execute(const CorrespondenceMaps3DSequence& inputMatches, Poses3DSequence& outputTransforms, bool& success, float& error)
	{
	dfn->matchesInput(inputMatches);
	dfn->process();
	Copy( dfn->transformsOutput(), outputTransforms);
	success = dfn->successOutput();
	error = dfn->errorOutput();
	}
}

/** @} */
