/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Triangulation.cpp
 * @date 29/01/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the Triangulation class.
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
#include "Triangulation.hpp"
#include <Errors/Assert.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include "opencv2/calib3d.hpp"
#include <SupportTypes.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Eigen/Geometry>
#include <Transform3DToMatConverter.hpp>

#include <stdlib.h>
#include <fstream>


using namespace Common;
using namespace Converters;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace SupportTypes;
using namespace BaseTypesWrapper;
using namespace MatrixWrapper;

namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
Triangulation::Triangulation()
	{
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "FocalLengthX", parameters.firstCameraMatrix.focalLengthX, DEFAULT_PARAMETERS.firstCameraMatrix.focalLengthX);
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "FocalLengthY", parameters.firstCameraMatrix.focalLengthY, DEFAULT_PARAMETERS.firstCameraMatrix.focalLengthY);
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "PrinciplePointX", parameters.firstCameraMatrix.principlePoint.x, DEFAULT_PARAMETERS.firstCameraMatrix.principlePoint.x);
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "PrinciplePointY", parameters.firstCameraMatrix.principlePoint.y, DEFAULT_PARAMETERS.firstCameraMatrix.principlePoint.y);
	parametersHelper.AddParameter<double>("SecondCameraMatrix", "FocalLengthX", parameters.secondCameraMatrix.focalLengthX, DEFAULT_PARAMETERS.secondCameraMatrix.focalLengthX);
	parametersHelper.AddParameter<double>("SecondCameraMatrix", "FocalLengthY", parameters.secondCameraMatrix.focalLengthY, DEFAULT_PARAMETERS.secondCameraMatrix.focalLengthY);
	parametersHelper.AddParameter<double>("SecondCameraMatrix", "PrinciplePointX", parameters.secondCameraMatrix.principlePoint.x, DEFAULT_PARAMETERS.secondCameraMatrix.principlePoint.x);
	parametersHelper.AddParameter<double>("SecondCameraMatrix", "PrinciplePointY", parameters.secondCameraMatrix.principlePoint.y, DEFAULT_PARAMETERS.secondCameraMatrix.principlePoint.y);

	firstCameraMatrix = ConvertToMat(DEFAULT_PARAMETERS.firstCameraMatrix);
	secondCameraMatrix = ConvertToMat(DEFAULT_PARAMETERS.secondCameraMatrix);

	configurationFilePath = "";
	}

Triangulation::~Triangulation()
	{

	}

void Triangulation::configure()
	{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

	firstCameraMatrix = ConvertToMat(parameters.firstCameraMatrix);
	secondCameraMatrix = ConvertToMat(parameters.secondCameraMatrix);
	}

void Triangulation::process() 
	{
	ValidateInputs(inCorrespondenceMap, inPose);
	cv::Mat projectionMatrix = ConversionCache<Pose3DConstPtr, cv::Mat, Transform3DToMatConverter>::Convert(inPose);
	cv::Mat pointsVectorAtSource = ConvertAtPose(inCorrespondenceMap, SOURCE_CAMERA);
	cv::Mat pointsVectorAtSink = ConvertAtPose(inCorrespondenceMap, SINK_CAMERA);
	cv::Mat uniformPointCloudMatrix = Triangulate(projectionMatrix, pointsVectorAtSource, pointsVectorAtSink);
	outPointCloud = Convert(uniformPointCloudMatrix);
	}

const Triangulation::TriangulationOptionsSet Triangulation::DEFAULT_PARAMETERS =
	{
	.firstCameraMatrix =
		{
		.focalLengthX = 1.0,
		.focalLengthY = 1.0,
		.principlePoint = cv::Point2d(0, 0)
		},
	.secondCameraMatrix =
		{
		.focalLengthX = 1.0,
		.focalLengthY = 1.0,
		.principlePoint = cv::Point2d(0, 0)
		}
	};

cv::Mat Triangulation::ConvertToMat(CameraMatrix cameraMatrix)
	{
	cv::Mat conversion(3, 3, CV_32FC1, cv::Scalar(0));
	conversion.at<float>(0,0) = cameraMatrix.focalLengthX;
	conversion.at<float>(1,1) = cameraMatrix.focalLengthY;
	conversion.at<float>(0,2) = cameraMatrix.principlePoint.x;
	conversion.at<float>(1,2) = cameraMatrix.principlePoint.y;
	conversion.at<float>(2,2) = 1.0;
	return conversion;
	}

cv::Mat Triangulation::Triangulate(cv::Mat projectionMatrix, cv::Mat pointsVectorAtSource, cv::Mat pointsVectorAtSink)
	{
	//The first camera projection matrix is [I|0] according to Result 9.14 of Multiple view geometry in computer vision. Richard Hartley, Andrew Zisserman.
	cv::Mat identityProjection(3, 4, CV_32FC1, cv::Scalar(0) );
	identityProjection.at<float>(0,0) = 1;
	identityProjection.at<float>(1,1) = 1;
	identityProjection.at<float>(2,2) = 1;

	cv::Mat homogeneousPointCloudMatrix;
	cv::triangulatePoints(firstCameraMatrix * identityProjection, secondCameraMatrix * projectionMatrix, pointsVectorAtSource, pointsVectorAtSink, homogeneousPointCloudMatrix);
	//cv::sfm::triangulatePoints(pointsVectorAtSource, projectionMatrix, homogeneousPointCloudMatrix);
	return homogeneousPointCloudMatrix;
	}

cv::Mat Triangulation::ConvertAtPose(CorrespondenceMap2DConstPtr correspondenceMap, CAMERA_TYPE cameraPoseIdentifier)
	{
	cv::Mat pointsVector(2, GetNumberOfCorrespondences(*correspondenceMap), CV_32FC1);

	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D point;
		if (cameraPoseIdentifier == SOURCE_CAMERA)
			{
			point = GetSource(*correspondenceMap, correspondenceIndex);
			}
		else
			{
			point = GetSink(*correspondenceMap, correspondenceIndex);
			}
		pointsVector.at<float>(0, correspondenceIndex) = point.x;
		pointsVector.at<float>(1, correspondenceIndex) = point.y;		
		}

	return pointsVector;
	}

PointCloudConstPtr Triangulation::Convert(cv::Mat homogeneousPointCloudMatrix)
	{
	static const float EPSILON = 1e-6;
	PointCloudPtr pointCloud = new PointCloud();
	ClearPoints(*pointCloud);

	for(unsigned pointIndex = 0; pointIndex < homogeneousPointCloudMatrix.cols; pointIndex++)
		{
		float homogeneousPointX = homogeneousPointCloudMatrix.at<float>(0, pointIndex);
		float homogeneousPointY = homogeneousPointCloudMatrix.at<float>(1, pointIndex);
		float homogeneousPointZ = homogeneousPointCloudMatrix.at<float>(2, pointIndex);
		float homogeneousPointFactor = homogeneousPointCloudMatrix.at<float>(3, pointIndex);
		if ( std::abs(homogeneousPointFactor) < EPSILON)
			{
			//AddPoint(*pointCloud, 0, 0, 0);
			}
		else
			{
			if (homogeneousPointZ/homogeneousPointFactor > 0)
				{
				AddPoint(*pointCloud, homogeneousPointX/homogeneousPointFactor, homogeneousPointY/homogeneousPointFactor, homogeneousPointZ/homogeneousPointFactor);
				}
			}
		}
	return pointCloud;
	}

void Triangulation::ValidateParameters()
	{
	ASSERT(parameters.firstCameraMatrix.focalLengthX > 0 && parameters.firstCameraMatrix.focalLengthY > 0, "EssentialMatrixRansac Configuration Error: focalLength is not positive");
	ASSERT(parameters.secondCameraMatrix.focalLengthX > 0 && parameters.secondCameraMatrix.focalLengthY > 0, "EssentialMatrixRansac Configuration Error: focalLength is not positive");
	}

void Triangulation::ValidateInputs(CorrespondenceMap2DConstPtr correspondenceMap, Pose3DConstPtr pose)
	{

	}


}


/** @} */
