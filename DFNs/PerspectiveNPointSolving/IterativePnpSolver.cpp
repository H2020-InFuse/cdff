/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file IterativePnpSolver.cpp
 * @date 20/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the IterativePnpSolver class.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "IterativePnpSolver.hpp"
#include <Errors/Assert.hpp>
#include <MatToTransform3DConverter.hpp>
#include <VisualPointFeatureVector2DToMatConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Macros/YamlcppMacros.hpp>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <stdlib.h>
#include <fstream>
#include <Eigen/Geometry>

using namespace Converters;
using namespace Common;

namespace dfn_ci {

using namespace VisualPointFeatureVector2DWrapper;
using namespace PointCloudWrapper;
using namespace PoseWrapper;



/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
IterativePnpSolver::IterativePnpSolver()
	{
	parametersHelper.AddParameter<float>("CameraMatrix", "FocalLengthX", parameters.cameraMatrix.focalLengthX, DEFAULT_PARAMETERS.cameraMatrix.focalLengthX); 
	parametersHelper.AddParameter<float>("CameraMatrix", "FocalLengthY", parameters.cameraMatrix.focalLengthY, DEFAULT_PARAMETERS.cameraMatrix.focalLengthY); 
	parametersHelper.AddParameter<float>("CameraMatrix", "PrinciplePointX", parameters.cameraMatrix.principlePointX, DEFAULT_PARAMETERS.cameraMatrix.principlePointX); 
	parametersHelper.AddParameter<float>("CameraMatrix", "PrinciplePointY", parameters.cameraMatrix.principlePointY, DEFAULT_PARAMETERS.cameraMatrix.principlePointY); 

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
	cv::Mat pointCloud = Convert(inPointCloud);
	cv::Mat featuresMatrix = ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Convert(inCameraFeaturesVector);
	ValidateInputs(pointCloud, featuresMatrix);

	cv::Mat pose = ComputePose(pointCloud, featuresMatrix, outSuccess);
	
	if (outSuccess)
		{
		outPose = ConversionCache<cv::Mat, Pose3DConstPtr, MatToPose3DConverter>::Convert(pose);
		}
	else
		{
		outPose = new Pose3D();
		}
	}

const IterativePnpSolver::IterativePnpOptionsSet IterativePnpSolver::DEFAULT_PARAMETERS =
	{
	.cameraMatrix =
		{
		.focalLengthX = 1,
		.focalLengthY = 1,
		.principlePointX = 0,
		.principlePointY = 0
		}
	};


cv::Mat IterativePnpSolver::ComputePose(cv::Mat pointCloud, cv::Mat featuresMatrix, bool& success)
	{
	cv::Mat cameraMatrix(3, 3, CV_32FC1, cv::Scalar(0));
	cameraMatrix.at<float>(0,0) = parameters.cameraMatrix.focalLengthX;
	cameraMatrix.at<float>(1,1) = parameters.cameraMatrix.focalLengthY;
	cameraMatrix.at<float>(0,2) = parameters.cameraMatrix.principlePointX;
	cameraMatrix.at<float>(1,2) = parameters.cameraMatrix.principlePointY;
	cameraMatrix.at<float>(2,2) = 1;

	cv::Mat rotationVector, translationVector;
	success = cv::solvePnP
		(
		pointCloud,
		featuresMatrix,
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

cv::Mat IterativePnpSolver::Convert(PointCloudWrapper::PointCloudConstPtr pointCloud)
	{
	unsigned numberOfPoints = GetNumberOfPoints(*pointCloud);
	cv::Mat cvPointCloud(numberOfPoints, 3, CV_32FC1);
	
	for(unsigned pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		cvPointCloud.at<float>(pointIndex, 0) = GetXCoordinate(*pointCloud, pointIndex);
		cvPointCloud.at<float>(pointIndex, 1) = GetYCoordinate(*pointCloud, pointIndex);
		cvPointCloud.at<float>(pointIndex, 2) = GetZCoordinate(*pointCloud, pointIndex);
		}

	return cvPointCloud;
	}


void IterativePnpSolver::ValidateParameters()
	{
	ASSERT(parameters.cameraMatrix.focalLengthX > 0 && parameters.cameraMatrix.focalLengthY > 0, "IterativePnpSolver Error: focalLength has to be positive");
	}

void IterativePnpSolver::ValidateInputs(cv::Mat pointCloud, cv::Mat featuresMatrix)
	{
	ASSERT( pointCloud.rows == featuresMatrix.rows, "IterativePnpSolver Error: point cloud and features set have different number of points");
	}

}


/** @} */
