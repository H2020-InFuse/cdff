/**
 * @addtogroup DFNs
 * @{
 */

#include "BundleAdjustmentExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(BundleAdjustmentInterface* dfn, CorrespondenceMaps2DSequenceConstPtr inputMatches, Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	Execute(dfn, *inputMatches, outputTransforms, success, error);
	}

void Execute(BundleAdjustmentInterface* dfn, CorrespondenceMaps2DSequenceConstPtr inputMatches, Poses3DSequencePtr outputTransforms, bool& success, float& error)
	{
	ASSERT(outputTransforms != NULL, "BundleAdjustmentExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputMatches, *outputTransforms, success, error);
	}

void Execute(BundleAdjustmentInterface* dfn, const CorrespondenceMaps2DSequence& inputMatches, Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	ASSERT( dfn!= NULL, "BundleAdjustmentExecutor, input dfn is null");
	ASSERT( outputTransforms == NULL, "BundleAdjustmentExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->correspondenceMapsSequenceInput(inputMatches);
	dfn->process();
	outputTransforms = & ( dfn->posesSequenceOutput() );
	success = dfn->successOutput();
	error = dfn->errorOutput();
	}

void Execute(BundleAdjustmentInterface* dfn, const CorrespondenceMaps2DSequence& inputMatches, Poses3DSequence& outputTransforms, bool& success, float& error)
	{
	ASSERT( dfn!= NULL, "BundleAdjustmentExecutor, input dfn is null");
	dfn->correspondenceMapsSequenceInput(inputMatches);
	dfn->process();
	Copy( dfn->posesSequenceOutput(), outputTransforms);
	success = dfn->successOutput();
	error = dfn->errorOutput();
	}

void Execute(BundleAdjustmentInterface* dfn, CorrespondenceMaps2DSequenceConstPtr inputMatches, Poses3DSequenceConstPtr poseGuess, PointCloudConstPtr cloudGuess, 
	Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	Execute(dfn, *inputMatches, *poseGuess, *cloudGuess, outputTransforms, success, error);
	}

void Execute(BundleAdjustmentInterface* dfn, CorrespondenceMaps2DSequenceConstPtr inputMatches, Poses3DSequenceConstPtr poseGuess, PointCloudConstPtr cloudGuess, 
	Poses3DSequencePtr outputTransforms, bool& success, float& error)
	{
	ASSERT(outputTransforms != NULL, "BundleAdjustmentExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputMatches, *poseGuess, *cloudGuess, *outputTransforms, success, error);
	}

void Execute(BundleAdjustmentInterface* dfn, const CorrespondenceMaps2DSequence& inputMatches, const Poses3DSequence& poseGuess, const PointCloud& cloudGuess, 
	Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	ASSERT( dfn!= NULL, "BundleAdjustmentExecutor, input dfn is null");
	ASSERT( outputTransforms == NULL, "BundleAdjustmentExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->correspondenceMapsSequenceInput(inputMatches);
	dfn->guessedPosesSequenceInput(poseGuess);
	dfn->guessedPointCloudInput(cloudGuess);
	dfn->process();
	outputTransforms = & ( dfn->posesSequenceOutput() );
	success = dfn->successOutput();
	error = dfn->errorOutput();
	}

void Execute(BundleAdjustmentInterface* dfn, const CorrespondenceMaps2DSequence& inputMatches, const Poses3DSequence& poseGuess, const PointCloud& cloudGuess, 
	Poses3DSequence& outputTransforms, bool& success, float& error)
	{
	ASSERT( dfn!= NULL, "BundleAdjustmentExecutor, input dfn is null");
	dfn->correspondenceMapsSequenceInput(inputMatches);
	dfn->guessedPosesSequenceInput(poseGuess);
	dfn->guessedPointCloudInput(cloudGuess);
	dfn->process();
	Copy( dfn->posesSequenceOutput(), outputTransforms);
	success = dfn->successOutput();
	error = dfn->errorOutput();
	}

}
}
}

/** @} */
