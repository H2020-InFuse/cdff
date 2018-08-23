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

BundleAdjustmentExecutor::BundleAdjustmentExecutor(BundleAdjustmentInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "Bundle Adjustment Executor: null dfn in input");
	}

BundleAdjustmentExecutor::~BundleAdjustmentExecutor()
	{

	}

void BundleAdjustmentExecutor::Execute(CorrespondenceMaps2DSequenceConstPtr inputMatches, Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	Execute(*inputMatches, outputTransforms, success, error);
	}

void BundleAdjustmentExecutor::Execute(CorrespondenceMaps2DSequenceConstPtr inputMatches, Poses3DSequencePtr outputTransforms, bool& success, float& error)
	{
	ASSERT(outputTransforms != NULL, "BundleAdjustmentExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputMatches, *outputTransforms, success, error);
	}

void BundleAdjustmentExecutor::Execute(const CorrespondenceMaps2DSequence& inputMatches, Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	ASSERT( outputTransforms == NULL, "BundleAdjustmentExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->correspondenceMapsSequenceInput(inputMatches);
	dfn->process();
	outputTransforms = & ( dfn->posesSequenceOutput() );
	success = dfn->successOutput();
	error = dfn->errorOutput();
	}

void BundleAdjustmentExecutor::Execute(const CorrespondenceMaps2DSequence& inputMatches, Poses3DSequence& outputTransforms, bool& success, float& error)
	{
	dfn->correspondenceMapsSequenceInput(inputMatches);
	dfn->process();
	Copy( dfn->posesSequenceOutput(), outputTransforms);
	success = dfn->successOutput();
	error = dfn->errorOutput();
	}

void BundleAdjustmentExecutor::Execute(CorrespondenceMaps2DSequenceConstPtr inputMatches, Poses3DSequenceConstPtr poseGuess, PointCloudConstPtr cloudGuess, 
	Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	Execute(*inputMatches, *poseGuess, *cloudGuess, outputTransforms, success, error);
	}

void BundleAdjustmentExecutor::Execute(CorrespondenceMaps2DSequenceConstPtr inputMatches, Poses3DSequenceConstPtr poseGuess, PointCloudConstPtr cloudGuess, 
	Poses3DSequencePtr outputTransforms, bool& success, float& error)
	{
	ASSERT(outputTransforms != NULL, "BundleAdjustmentExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputMatches, *poseGuess, *cloudGuess, *outputTransforms, success, error);
	}

void BundleAdjustmentExecutor::Execute(const CorrespondenceMaps2DSequence& inputMatches, const Poses3DSequence& poseGuess, const PointCloud& cloudGuess, 
	Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error)
	{
	ASSERT( outputTransforms == NULL, "BundleAdjustmentExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->correspondenceMapsSequenceInput(inputMatches);
	dfn->guessedPosesSequenceInput(poseGuess);
	dfn->guessedPointCloudInput(cloudGuess);
	dfn->process();
	outputTransforms = & ( dfn->posesSequenceOutput() );
	success = dfn->successOutput();
	error = dfn->errorOutput();
	}

void BundleAdjustmentExecutor::Execute(const CorrespondenceMaps2DSequence& inputMatches, const Poses3DSequence& poseGuess, const PointCloud& cloudGuess, 
	Poses3DSequence& outputTransforms, bool& success, float& error)
	{
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

/** @} */
