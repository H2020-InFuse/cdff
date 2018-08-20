/**
 * @addtogroup DFNs
 * @{
 */

#include "Registration3DExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace PointCloudWrapper;
using namespace PoseWrapper;

namespace CDFF
{
namespace DFN
{

Registration3DExecutor::Registration3DExecutor(Registration3DInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "Registration3DExecutor: null dfn in input");
	}

Registration3DExecutor::~Registration3DExecutor()
	{

	}

void Registration3DExecutor::Execute(PointCloudConstPtr inputSourceCloud, PointCloudConstPtr inputSinkCloud, Pose3DConstPtr& outputTransform, bool& success)
	{
	Execute(*inputSourceCloud, *inputSinkCloud, outputTransform, success);
	}

void Registration3DExecutor::Execute(PointCloudConstPtr inputSourceCloud, PointCloudConstPtr inputSinkCloud, Pose3DPtr outputTransform, bool& success)
	{
	ASSERT(outputTransform != NULL, "Registration3DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputSourceCloud, *inputSinkCloud, *outputTransform, success);
	}

void Registration3DExecutor::Execute(const PointCloud& inputSourceCloud, const PointCloud& inputSinkCloud, Pose3DConstPtr& outputTransform, bool& success)
	{
	ASSERT( outputTransform == NULL, "Registration3DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->sourceCloudInput(inputSourceCloud);
	dfn->sinkCloudInput(inputSinkCloud);
	bool guessInput = false;
	dfn->useGuessInput(guessInput);
	dfn->process();
	outputTransform = & ( dfn->transformOutput() );
	success = dfn->successOutput();
	}

void Registration3DExecutor::Execute(const PointCloud& inputSourceCloud, const PointCloud& inputSinkCloud, Pose3D& outputTransform, bool& success)
	{
	dfn->sourceCloudInput(inputSourceCloud);
	dfn->sinkCloudInput(inputSinkCloud);
	bool guessInput = false;
	dfn->useGuessInput(guessInput);
	dfn->process();
	Copy( dfn->transformOutput(), outputTransform);
	success = dfn->successOutput();
	}

void Registration3DExecutor::Execute(PointCloudConstPtr inputSourceCloud, PointCloudConstPtr inputSinkCloud, Pose3DConstPtr poseGuess, Pose3DConstPtr& outputTransform, bool& success)
	{
	Execute(*inputSourceCloud, *inputSinkCloud, *poseGuess, outputTransform, success);
	}

void Registration3DExecutor::Execute(PointCloudConstPtr inputSourceCloud, PointCloudConstPtr inputSinkCloud, Pose3DConstPtr poseGuess, Pose3DPtr outputTransform, bool& success)
	{
	ASSERT(outputTransform != NULL, "Registration3DExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputSourceCloud, *inputSinkCloud, *poseGuess, *outputTransform, success);
	}

void Registration3DExecutor::Execute(const PointCloud& inputSourceCloud, const PointCloud& inputSinkCloud, const Pose3D& poseGuess, Pose3DConstPtr& outputTransform, bool& success)
	{
	ASSERT( outputTransform == NULL, "Registration3DExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->sourceCloudInput(inputSourceCloud);
	dfn->sinkCloudInput(inputSinkCloud);
	bool guessInput = true;
	dfn->useGuessInput(guessInput);
	dfn->transformGuessInput(poseGuess);
	dfn->process();
	outputTransform = & ( dfn->transformOutput() );
	success = dfn->successOutput();
	}

void Registration3DExecutor::Execute(const PointCloud& inputSourceCloud, const PointCloud& inputSinkCloud, const Pose3D& poseGuess, Pose3D& outputTransform, bool& success)
	{
	dfn->sourceCloudInput(inputSourceCloud);
	dfn->sinkCloudInput(inputSinkCloud);
	bool guessInput = true;
	dfn->useGuessInput(guessInput);
	dfn->transformGuessInput(poseGuess);
	dfn->process();
	Copy( dfn->transformOutput(), outputTransform);
	success = dfn->successOutput();
	}

}
}

/** @} */
