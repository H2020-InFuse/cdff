/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file EssentialMatrixComputation.cpp
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
#include "EssentialMatrixComputation.hpp"
#include <Errors/Assert.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Macros/YamlcppMacros.hpp>
#include "opencv2/calib3d.hpp"
#include <Eigen/Geometry>
#include <MatToTransform3DConverter.hpp>

#include <stdlib.h>
#include <fstream>

using namespace Common;
using namespace Helpers;

namespace dfn_ci {

using namespace PoseWrapper;
using namespace MatrixWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace Converters;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
EssentialMatrixComputation::EssentialMatrixComputation()
	{
	parametersHelper.AddParameter<int>("GeneralParameters", "NumberOfTestPoints", parameters.numberOfTestPoints, DEFAULT_PARAMETERS.numberOfTestPoints);
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

EssentialMatrixComputation::~EssentialMatrixComputation()
	{

	}

void EssentialMatrixComputation::configure()
	{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

	firstCameraMatrix = ConvertToMat(parameters.firstCameraMatrix);
	secondCameraMatrix = ConvertToMat(parameters.secondCameraMatrix);
	}


void EssentialMatrixComputation::process() 
	{
	cv::Mat correspondenceMap = Convert(inCorrespondenceMap);
	cv::Mat fundamentalMatrix = ConvertToMat(inFundamentalMatrix);
	ValidateInputs(fundamentalMatrix, correspondenceMap);

	std::vector<cv::Mat> transformsList = ComputeTransformMatrix(fundamentalMatrix);
	int validTransformIndex = FindValidTransform(transformsList, correspondenceMap);

	if (validTransformIndex < 0)
		{
		outTransform =  new Transform3D();
		outSuccess = false;
		}	
	else
		{
		outTransform = ConversionCache<cv::Mat, Pose3DConstPtr, MatToTransform3DConverter>::Convert(transformsList.at(validTransformIndex));
		outSuccess = true;
		}
	}


const EssentialMatrixComputation::EssentialMatrixComputationOptionsSet EssentialMatrixComputation::DEFAULT_PARAMETERS =
	{
	.numberOfTestPoints = 20,
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

cv::Mat EssentialMatrixComputation::ConvertToMat(CameraMatrix cameraMatrix)
	{
	cv::Mat conversion(3, 3, CV_32FC1, cv::Scalar(0));
	conversion.at<float>(0,0) = cameraMatrix.focalLengthX;
	conversion.at<float>(1,1) = cameraMatrix.focalLengthY;
	conversion.at<float>(0,2) = cameraMatrix.principlePoint.x;
	conversion.at<float>(1,2) = cameraMatrix.principlePoint.y;
	conversion.at<float>(2,2) = 1.0;
	return conversion;
	}

cv::Mat EssentialMatrixComputation::ConvertToMat(MatrixWrapper::Matrix3dConstPtr matrix)
	{
	cv::Mat cvMatrix(3, 3, CV_32FC1);
	for(unsigned row = 0; row < 3; row++)
		{
		for(unsigned column = 0; column < 3; column++)
			{
			cvMatrix.at<float>(row,column) = GetElement(*matrix, row, column);
			}
		}
	return cvMatrix;
	}

cv::Mat EssentialMatrixComputation::Convert(CorrespondenceMap2DConstPtr correspondenceMap)
	{
	cv::Mat cvCorrespondenceMap(4, GetNumberOfCorrespondences(*correspondenceMap), CV_32FC1);
	for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
		{
		BaseTypesWrapper::Point2D firstPoint = GetSource(*correspondenceMap, correspondenceIndex);
		BaseTypesWrapper::Point2D secondPoint = GetSink(*correspondenceMap, correspondenceIndex);
		cvCorrespondenceMap.at<float>(0, correspondenceIndex) = firstPoint.x;
		cvCorrespondenceMap.at<float>(1, correspondenceIndex) = firstPoint.y;
		cvCorrespondenceMap.at<float>(2, correspondenceIndex) = secondPoint.x;
		cvCorrespondenceMap.at<float>(3, correspondenceIndex) = secondPoint.y;
		}
	return cvCorrespondenceMap;
	}

std::vector<cv::Mat> EssentialMatrixComputation::ComputeTransformMatrix(cv::Mat fundamentalMatrix)
	{
	cv::Mat essentialMatrix = secondCameraMatrix.t() * fundamentalMatrix * firstCameraMatrix;

	cv::Mat firstRotationMatrix, secondRotationMatrix, translationMatrix;
	cv::decomposeEssentialMat(essentialMatrix, firstRotationMatrix, secondRotationMatrix, translationMatrix);

	std::vector<cv::Mat> projectionMatricesList(4);
	cv::hconcat(firstRotationMatrix, translationMatrix, projectionMatricesList[0]);
	cv::hconcat(firstRotationMatrix, -translationMatrix, projectionMatricesList[1]);
	cv::hconcat(secondRotationMatrix, translationMatrix, projectionMatricesList[2]);
	cv::hconcat(secondRotationMatrix, -translationMatrix, projectionMatricesList[3]);

	return projectionMatricesList;
	}

int EssentialMatrixComputation::FindValidTransform(std::vector<cv::Mat> projectionsList, cv::Mat correspondenceMap)
	{
	static const float EPSILON = 1e-5;

	cv::Mat identityProjection(3, 4, CV_32FC1, cv::Scalar(0));
	identityProjection.at<float>(0,0) = 1;
	identityProjection.at<float>(1,1) = 1;
	identityProjection.at<float>(2,2) = 1;

	unsigned testPointsNumber = (correspondenceMap.cols > parameters.numberOfTestPoints) ? parameters.numberOfTestPoints : correspondenceMap.cols;
	int validMatrixIndex = -1;
	bool validMatrixFound = false;
	for(unsigned matrixIndex = 0; matrixIndex < projectionsList.size(); matrixIndex++)
		{
		cv::Mat currentProjectionMatrix = projectionsList.at(matrixIndex);
		double rotationDeterminant = cv::determinant( currentProjectionMatrix( cv::Rect(0,0,2,2) ));
		bool orientationPreserved = std::abs(rotationDeterminant - 1) < EPSILON;
		bool matrixIsValid = orientationPreserved;
		
		if (matrixIsValid)
			{
			cv::Mat testPointCloudMatrix;
			cv::triangulatePoints
				(
				firstCameraMatrix * identityProjection, 
				secondCameraMatrix * currentProjectionMatrix, 
				correspondenceMap(cv::Rect(0, 0, testPointsNumber, 2 )),
				correspondenceMap(cv::Rect(0, 2, testPointsNumber, 2 )),
				testPointCloudMatrix
				);

			for(unsigned pointIndex = 0; pointIndex < testPointCloudMatrix.cols && matrixIsValid; pointIndex++)
				{
				float z = testPointCloudMatrix.at<float>(2, pointIndex) / testPointCloudMatrix.at<float>(3, pointIndex);
				matrixIsValid = (z == z) && (!std::isinf(z)) && (z >= 0);
				}
			}

		if (matrixIsValid)
			{
			if (validMatrixFound)
				{
				return -1;
				}
			validMatrixFound = true;
			validMatrixIndex = matrixIndex;
			}
		}

	return validMatrixIndex;
	}


void EssentialMatrixComputation::ValidateParameters()
	{
	ASSERT(parameters.numberOfTestPoints > 0, "EssentialMatrixComputation Configuration Error: number of test points has to be positive");
	ASSERT(parameters.firstCameraMatrix.focalLengthX > 0 && parameters.firstCameraMatrix.focalLengthY > 0, "EssentialMatrixComputation Configuration Error: focalLength is not positive");
	ASSERT(parameters.secondCameraMatrix.focalLengthX > 0 && parameters.secondCameraMatrix.focalLengthY > 0, "EssentialMatrixComputation Configuration Error: focalLength is not positive");
	}

void EssentialMatrixComputation::ValidateInputs(cv::Mat fundamentalMatrix, cv::Mat correspondenceMap)
	{

	}
}


/** @} */
