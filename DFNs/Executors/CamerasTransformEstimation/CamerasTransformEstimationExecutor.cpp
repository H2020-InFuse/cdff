/**
 * @addtogroup DFNs
 * @{
 */

#include "CamerasTransformEstimationExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace MatrixWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;

namespace CDFF
{
namespace DFN
{
namespace Executors
{

void Execute(CamerasTransformEstimationInterface* dfn, Matrix3dConstPtr inputMatrix, CorrespondenceMap2DConstPtr inputMatches, Pose3DConstPtr& outputTransform, bool& success)
	{
	Execute(dfn, *inputMatrix, *inputMatches, outputTransform, success);
	}

void Execute(CamerasTransformEstimationInterface* dfn, Matrix3dConstPtr inputMatrix, CorrespondenceMap2DConstPtr inputMatches, Pose3DPtr outputTransform, bool& success)
	{
	ASSERT(outputTransform != NULL, "CamerasTransformEstimationExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(dfn, *inputMatrix, *inputMatches, *outputTransform, success);
	}

void Execute(CamerasTransformEstimationInterface* dfn, const Matrix3d& inputMatrix, const CorrespondenceMap2D& inputMatches, Pose3DConstPtr& outputTransform, bool& success)
	{
	ASSERT( dfn!= NULL, "CamerasTransformEstimationExecutor, input dfn is null");
	ASSERT( outputTransform == NULL, "CamerasTransformEstimationExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->fundamentalMatrixInput(inputMatrix);
	dfn->matchesInput(inputMatches);
	dfn->process();
	outputTransform = & ( dfn->transformOutput() );
	success = dfn->successOutput();
	}

void Execute(CamerasTransformEstimationInterface* dfn, const Matrix3d& inputMatrix, const CorrespondenceMap2D& inputMatches, Pose3D& outputTransform, bool& success)
	{
	ASSERT( dfn!= NULL, "CamerasTransformEstimationExecutor, input dfn is null");
	dfn->fundamentalMatrixInput(inputMatrix);
	dfn->matchesInput(inputMatches);
	dfn->process();
	Copy( dfn->transformOutput(), outputTransform);
	success = dfn->successOutput();
	}

}
}
}

/** @} */
