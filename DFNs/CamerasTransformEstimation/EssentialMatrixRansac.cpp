/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file EssentialMatrixRansac.cpp
 * @date 31/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the Ransac Method for essential matrix computation.
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
#include "EssentialMatrixRansac.hpp"
#include <Errors/Assert.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Macros/YamlcppMacros.hpp>
#include "opencv2/calib3d.hpp"
#include <Eigen/Geometry>

#include <stdlib.h>
#include <fstream>

using namespace Common;
using namespace Helpers;

namespace dfn_ci {

using namespace PoseWrapper;
using namespace CorrespondenceMap2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
EssentialMatrixRansac::EssentialMatrixRansac()
	{
	parametersHelper.AddParameter<double>("GeneralParameters", "OutlierThreshold", parameters.outlierThreshold, DEFAULT_PARAMETERS.outlierThreshold);
	parametersHelper.AddParameter<double>("GeneralParameters", "Confidence", parameters.confidence, DEFAULT_PARAMETERS.confidence);
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "FocalLengthX", parameters.firstCameraMatrix.focalLengthX, DEFAULT_PARAMETERS.firstCameraMatrix.focalLengthX);
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "FocalLengthY", parameters.firstCameraMatrix.focalLengthY, DEFAULT_PARAMETERS.firstCameraMatrix.focalLengthY);
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "PrinciplePointX", parameters.firstCameraMatrix.principlePoint.x, DEFAULT_PARAMETERS.firstCameraMatrix.principlePoint.x);
	parametersHelper.AddParameter<double>("FirstCameraMatrix", "PrinciplePointY", parameters.firstCameraMatrix.principlePoint.y, DEFAULT_PARAMETERS.firstCameraMatrix.principlePoint.y);
	parametersHelper.AddParameter<double>("SecondCameraMatrix", "FocalLengthX", parameters.secondCameraMatrix.focalLengthX, DEFAULT_PARAMETERS.secondCameraMatrix.focalLengthX);
	parametersHelper.AddParameter<double>("SecondCameraMatrix", "FocalLengthY", parameters.secondCameraMatrix.focalLengthY, DEFAULT_PARAMETERS.secondCameraMatrix.focalLengthY);
	parametersHelper.AddParameter<double>("SecondCameraMatrix", "PrinciplePointX", parameters.secondCameraMatrix.principlePoint.x, DEFAULT_PARAMETERS.secondCameraMatrix.principlePoint.x);
	parametersHelper.AddParameter<double>("SecondCameraMatrix", "PrinciplePointY", parameters.secondCameraMatrix.principlePoint.y, DEFAULT_PARAMETERS.secondCameraMatrix.principlePoint.y);

	configurationFilePath = "";
	}

EssentialMatrixRansac::~EssentialMatrixRansac()
	{

	}

void EssentialMatrixRansac::configure()
	{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
	}


void EssentialMatrixRansac::process() 
	{
	std::vector<cv::Point2d> firstImagePointsVector;
	std::vector<cv::Point2d> secondImagePointsVector;
	Convert(inCorrespondenceMap, firstImagePointsVector, secondImagePointsVector);
	ValidateInputs(firstImagePointsVector, secondImagePointsVector);
	cv::Mat transformMatrix = ComputeTransformMatrix(firstImagePointsVector, secondImagePointsVector);
	
	if (transformMatrix.rows == 0 && transformMatrix.cols == 0)
		{
		outTransform =  new Transform3D();
		outSuccess = false;
		}	
	else
		{
		outTransform = Convert(transformMatrix);
		outSuccess = true;
		}
	}


const EssentialMatrixRansac::EssentialMatrixRansacOptionsSet EssentialMatrixRansac::DEFAULT_PARAMETERS =
	{
	.outlierThreshold = 1,
	.confidence = 0.99,
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

cv::Mat EssentialMatrixRansac::ConvertToMat(CameraMatrix cameraMatrix)
	{
	cv::Mat conversion(3, 3, CV_64FC1, cv::Scalar(0));
	conversion.at<double>(0,0) = cameraMatrix.focalLengthX;
	conversion.at<double>(1,1) = cameraMatrix.focalLengthY;
	conversion.at<double>(0,2) = cameraMatrix.principlePoint.x;
	conversion.at<double>(1,2) = cameraMatrix.principlePoint.y;
	conversion.at<double>(2,2) = 1.0;
	return conversion;
	}

cv::Mat EssentialMatrixRansac::ComputeTransformMatrix(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector)
	{
	static const int METHOD = CV_FM_RANSAC;	

	cv::Mat fundamentalMatrix = cv::findFundamentalMat
		(
		firstImagePointsVector,
		secondImagePointsVector,
		METHOD,
		parameters.outlierThreshold,
		parameters.confidence
		);

	if (fundamentalMatrix.cols == 0 && fundamentalMatrix.rows == 0)
		{
		return cv::Mat();
		}

	cv::Mat essentialMatrix = ComputeEssentialMatrix(fundamentalMatrix);
	/*cv::Mat essentialMatrix = cv::findEssentialMat
		(
		firstImagePointsVector,
		secondImagePointsVector,
		parameters.firstCameraMatrix.focalLengthX,
		parameters.firstCameraMatrix.principlePoint,
		cv::RANSAC,
		parameters.confidence,
		parameters.outlierThreshold
		);*/

	cv::Mat rotationMatrix1, rotationMatrix2, translationMatrix;
	cv::decomposeEssentialMat(essentialMatrix, rotationMatrix1, rotationMatrix2, translationMatrix);

	if (rotationMatrix1.cols == 0 && rotationMatrix1.rows == 0)
		{
		return cv::Mat();
		}

	cv::Mat transformMatrix(3, 4, CV_32FC1, cv::Scalar(0));
	rotationMatrix1.convertTo( transformMatrix(cv::Rect(0,0,3,3)), CV_32FC1 );
	translationMatrix.convertTo( transformMatrix(cv::Rect(3, 0, 1, 3)), CV_32FC1 );

	return transformMatrix;
	}

cv::Mat EssentialMatrixRansac::ComputeEssentialMatrix(cv::Mat fundamentalMatrix)
	{
	cv::Mat firstCameraMatrix = ConvertToMat(parameters.firstCameraMatrix);
	cv::Mat secondCameraMatrix = ConvertToMat(parameters.secondCameraMatrix);

	cv::Mat secondCameraMatrixTransposed;
	cv::transpose(secondCameraMatrix, secondCameraMatrixTransposed);
	cv::Mat essentialMatrix = secondCameraMatrixTransposed * fundamentalMatrix * firstCameraMatrix;

	return essentialMatrix;
	}

void EssentialMatrixRansac::Convert(CorrespondenceMap2DConstPtr correspondenceMap, std::vector<cv::Point2d>& firstImagePointsVector, std::vector<cv::Point2d>& secondImagePointsVector)
	{
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D firstPoint = GetSource(*correspondenceMap, correspondenceIndex);
		BaseTypesWrapper::Point2D secondPoint = GetSink(*correspondenceMap, correspondenceIndex);
		cv::Point2d firstCvPoint(firstPoint.x, firstPoint.y);
		cv::Point2d secondCvPoint(secondPoint.x, secondPoint.y);
		firstImagePointsVector.push_back(firstCvPoint);
		secondImagePointsVector.push_back(secondCvPoint);
		}
	}

Transform3DConstPtr EssentialMatrixRansac::Convert(cv::Mat transformMatrix)
	{
	ASSERT( transformMatrix.cols == 4 && transformMatrix.rows == 3, "EssentialMatrixRansac: unexpected transform matrix size in output");

	Eigen::Matrix3f eigenRotationMatrix;
	eigenRotationMatrix <<  transformMatrix.at<float>(0, 0), transformMatrix.at<float>(0, 1), transformMatrix.at<float>(0, 2),
				transformMatrix.at<float>(1, 0), transformMatrix.at<float>(1, 1), transformMatrix.at<float>(1, 2),
				transformMatrix.at<float>(2, 0), transformMatrix.at<float>(2, 1), transformMatrix.at<float>(2, 2);	
	Eigen::Quaternionf eigenRotation (eigenRotationMatrix);

	Transform3DPtr conversion = new Transform3D();
	SetPosition(*conversion, transformMatrix.at<float>(3, 0), transformMatrix.at<float>(3, 1), transformMatrix.at<float>(3, 2) );
	SetOrientation(*conversion, eigenRotation.x(), eigenRotation.y(), eigenRotation.z(), eigenRotation.w());

	return conversion;
	}

void EssentialMatrixRansac::ValidateParameters()
	{
	ASSERT(parameters.outlierThreshold >= 0, "EssentialMatrixRansac Configuration Error: outlierThreshold is negative");
	ASSERT(parameters.confidence >= 0 && parameters.confidence <= 1, "EssentialMatrixRansac Configuration Error: confidence should be a probability between 0 and 1");
	ASSERT(parameters.firstCameraMatrix.focalLengthX > 0 && parameters.firstCameraMatrix.focalLengthY > 0, "EssentialMatrixRansac Configuration Error: focalLength is not positive");
	ASSERT(parameters.secondCameraMatrix.focalLengthX > 0 && parameters.secondCameraMatrix.focalLengthY > 0, "EssentialMatrixRansac Configuration Error: focalLength is not positive");
	}

void EssentialMatrixRansac::ValidateInputs(const std::vector<cv::Point2d>& firstImagePointsVector, const std::vector<cv::Point2d>& secondImagePointsVector)
	{
	ASSERT(firstImagePointsVector.size() == secondImagePointsVector.size(), "EssentialMatrixRansac Error: Points vector do not have the same size");
	}
}


/** @} */
