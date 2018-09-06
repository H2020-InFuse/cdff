/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "IterativePnpSolver.hpp"

#include <Pose.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/opencv_modules.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace PoseWrapper;
using namespace PointCloudWrapper;

namespace CDFF
{
namespace DFN
{
namespace PerspectiveNPointSolving
{

IterativePnpSolver::IterativePnpSolver()
{
	parametersHelper.AddParameter<float>("CameraMatrix", "FocalLengthX", parameters.cameraMatrix.focalLengthX, DEFAULT_PARAMETERS.cameraMatrix.focalLengthX);
	parametersHelper.AddParameter<float>("CameraMatrix", "FocalLengthY", parameters.cameraMatrix.focalLengthY, DEFAULT_PARAMETERS.cameraMatrix.focalLengthY);
	parametersHelper.AddParameter<float>("CameraMatrix", "PrinciplePointX", parameters.cameraMatrix.principalPointX, DEFAULT_PARAMETERS.cameraMatrix.principalPointX);
	parametersHelper.AddParameter<float>("CameraMatrix", "PrinciplePointY", parameters.cameraMatrix.principalPointY, DEFAULT_PARAMETERS.cameraMatrix.principalPointY);

	configurationFilePath = "";
}

IterativePnpSolver::~IterativePnpSolver()
{
}

void IterativePnpSolver::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void IterativePnpSolver::process()
{
	// Read data from input ports
	cv::Mat points = Convert(&inPoints);
	cv::Mat projections = visualPointFeatureVector2DToMat.Convert(&inProjections);

	// Process data
	ValidateInputs(points, projections);
	cv::Mat camera = ComputePose(points, projections, outSuccess);

	// Write data to output port
	if (outSuccess)
	{
		Pose3DConstPtr tmp = matToPose3D.Convert(camera);
		Copy(*tmp, outCamera);
		delete tmp;
	}
}

const IterativePnpSolver::IterativePnpOptionsSet IterativePnpSolver::DEFAULT_PARAMETERS =
{
	//.cameraMatrix =
	{
		/*.focalLengthX =*/ 1,
		/*.focalLengthY =*/ 1,
		/*.principalPointX =*/ 0,
		/*.principalPointY =*/ 0
	}
};

cv::Mat IterativePnpSolver::ComputePose(cv::Mat points, cv::Mat projections, bool& success)
{
	cv::Mat cameraMatrix(3, 3, CV_32FC1, cv::Scalar(0));
	cameraMatrix.at<float>(0,0) = parameters.cameraMatrix.focalLengthX;
	cameraMatrix.at<float>(1,1) = parameters.cameraMatrix.focalLengthY;
	cameraMatrix.at<float>(0,2) = parameters.cameraMatrix.principalPointX;
	cameraMatrix.at<float>(1,2) = parameters.cameraMatrix.principalPointY;
	cameraMatrix.at<float>(2,2) = 1;

	cv::Mat rotationVector, translationVector;
	success = cv::solvePnP(
		points,
		projections,
		cameraMatrix,
		cv::Mat(),
		rotationVector,
		translationVector,
		false,
		cv::SOLVEPNP_ITERATIVE
	);

	cv::Mat rotationMatrix;
	cv::Rodrigues(rotationVector, rotationMatrix);

	cv::Mat poseMatrix;
	cv::hconcat(rotationMatrix, translationVector, poseMatrix);

	return poseMatrix;
}

cv::Mat IterativePnpSolver::Convert(PointCloudConstPtr pointCloud)
{
	unsigned numberOfPoints = GetNumberOfPoints(*pointCloud);
	cv::Mat cvPointCloud(numberOfPoints, 3, CV_32FC1);

	for (unsigned pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
	{
		cvPointCloud.at<float>(pointIndex, 0) = GetXCoordinate(*pointCloud, pointIndex);
		cvPointCloud.at<float>(pointIndex, 1) = GetYCoordinate(*pointCloud, pointIndex);
		cvPointCloud.at<float>(pointIndex, 2) = GetZCoordinate(*pointCloud, pointIndex);
	}

	return cvPointCloud;
}

void IterativePnpSolver::ValidateParameters()
{
	ASSERT(parameters.cameraMatrix.focalLengthX > 0 && parameters.cameraMatrix.focalLengthY > 0,
		"IterativePnpSolver Error: focal length is not strictly positive");
}

void IterativePnpSolver::ValidateInputs(cv::Mat points, cv::Mat projections)
{
	ASSERT(points.rows == projections.rows,
		"IterativePnpSolver Error: the points and their projections are in a different number");
}

}
}
}

/** @} */
