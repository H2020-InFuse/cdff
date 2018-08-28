/**
 * @addtogroup DFNs
 * @{
 */

#include "PerspectiveNPointSolvingExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace VisualPointFeatureVector2DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

namespace CDFF
{
namespace DFN
{

PerspectiveNPointSolvingExecutor::PerspectiveNPointSolvingExecutor(PerspectiveNPointSolvingInterface* dfn)
	{
	this->dfn = dfn;
	ASSERT(dfn != NULL, "PerspectiveNPointSolvingExecutor: null dfn in input");
	}

PerspectiveNPointSolvingExecutor::~PerspectiveNPointSolvingExecutor()
	{

	}

void PerspectiveNPointSolvingExecutor::Execute(PointCloudConstPtr inputCloud, VisualPointFeatureVector2DConstPtr inputKeypoints, PoseWrapper::Pose3DConstPtr& outputPose, bool& success)
	{
	Execute(*inputCloud, *inputKeypoints, outputPose, success);
	}

void PerspectiveNPointSolvingExecutor::Execute(PointCloudConstPtr inputCloud, VisualPointFeatureVector2DConstPtr inputKeypoints, PoseWrapper::Pose3DPtr outputPose, bool& success)
	{
	ASSERT(outputPose != NULL, "PerspectiveNPointSolvingExecutor, Calling NO instance creation Executor with a NULL pointer");
	Execute(*inputCloud, *inputKeypoints, *outputPose, success);
	}

void PerspectiveNPointSolvingExecutor::Execute(const PointCloud& inputCloud, const VisualPointFeatureVector2D& inputKeypoints, PoseWrapper::Pose3DConstPtr& outputPose, bool& success)
	{
	ASSERT( outputPose == NULL, "PerspectiveNPointSolvingExecutor, Calling instance creation executor with a non-NULL pointer");
	dfn->pointsInput(inputCloud);
	dfn->projectionsInput(inputKeypoints);
	dfn->process();
	outputPose = & ( dfn->cameraOutput() );
	success = dfn->successOutput();
	}

void PerspectiveNPointSolvingExecutor::Execute(const PointCloud& inputCloud, const VisualPointFeatureVector2D& inputKeypoints, PoseWrapper::Pose3D& outputPose, bool& success)
	{
	dfn->pointsInput(inputCloud);
	dfn->projectionsInput(inputKeypoints);
	dfn->process();
	Copy( dfn->cameraOutput(), outputPose);
	success = dfn->successOutput();
	}
}
}

/** @} */
