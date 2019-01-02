/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "EssentialMatrixDecomposition.hpp"

#include <Errors/Assert.hpp>
#include <Macros/YamlcppMacros.hpp>

#include <opencv2/calib3d.hpp>
#include <Eigen/Geometry>

#include <stdlib.h>
#include <fstream>

using namespace PoseWrapper;
using namespace MatrixWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace Converters;
using namespace Helpers;

namespace CDFF
{
namespace DFN
{
namespace CamerasTransformEstimation
{

EssentialMatrixDecomposition::EssentialMatrixDecomposition()
{
	parameters = DEFAULT_PARAMETERS;

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

EssentialMatrixDecomposition::~EssentialMatrixDecomposition()
{
}

void EssentialMatrixDecomposition::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

	firstCameraMatrix = ConvertToMat(parameters.firstCameraMatrix);
	secondCameraMatrix = ConvertToMat(parameters.secondCameraMatrix);
}

void EssentialMatrixDecomposition::process()
{
	// Read data from input port
	cv::Mat correspondenceMap = Convert(&inMatches);
	cv::Mat fundamentalMatrix = ConvertToMat(&inFundamentalMatrix);

	// Process data
	ValidateInputs(fundamentalMatrix, correspondenceMap);
	std::vector<cv::Mat> transformsList = ComputeTransformMatrix(fundamentalMatrix);
	int validTransformIndex = FindValidTransform(transformsList, correspondenceMap);

	// Write data to output port
	if (validTransformIndex < 0)
	{
		// outTransform =  new Transform3D();
		outSuccess = false;
	}
	else
	{
		Pose3DConstPtr tmp = matToTransform3DConverter.Convert(transformsList.at(validTransformIndex));
		Copy(*tmp, outTransform);
		delete(tmp);
		outSuccess = true;
	}
}

const EssentialMatrixDecomposition::EssentialMatrixDecompositionOptionsSet EssentialMatrixDecomposition::DEFAULT_PARAMETERS =
{
	/*.numberOfTestPoints =*/ 20,
	//.firstCameraMatrix =
	{
		/*.focalLengthX =*/ 1.0,
		/*.focalLengthY =*/ 1.0,
		/*.principlePoint =*/ cv::Point2d(0, 0)
	},
	//.secondCameraMatrix =
	{
		/*.focalLengthX =*/ 1.0,
		/*.focalLengthY =*/ 1.0,
		/*.principlePoint =*/ cv::Point2d(0, 0)
	}
};

cv::Mat EssentialMatrixDecomposition::ConvertToMat(CameraMatrix cameraMatrix)
{
	cv::Mat conversion(3, 3, CV_64FC1, cv::Scalar(0));
	conversion.at<double>(0,0) = cameraMatrix.focalLengthX;
	conversion.at<double>(1,1) = cameraMatrix.focalLengthY;
	conversion.at<double>(0,2) = cameraMatrix.principlePoint.x;
	conversion.at<double>(1,2) = cameraMatrix.principlePoint.y;
	conversion.at<double>(2,2) = 1.0;
	return conversion;
}

cv::Mat EssentialMatrixDecomposition::ConvertToMat(MatrixWrapper::Matrix3dConstPtr matrix)
{
	cv::Mat cvMatrix(3, 3, CV_64FC1);
	for (unsigned row = 0; row < 3; row++)
	{
		for (unsigned column = 0; column < 3; column++)
		{
			cvMatrix.at<double>(row,column) = GetElement(*matrix, row, column);
		}
	}
	return cvMatrix;
}

cv::Mat EssentialMatrixDecomposition::Convert(CorrespondenceMap2DConstPtr correspondenceMap)
{
	cv::Mat cvCorrespondenceMap(4, GetNumberOfCorrespondences(*correspondenceMap), CV_64FC1);
	for (int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(*correspondenceMap); correspondenceIndex++)
	{
		BaseTypesWrapper::Point2D firstPoint = GetSource(*correspondenceMap, correspondenceIndex);
		BaseTypesWrapper::Point2D secondPoint = GetSink(*correspondenceMap, correspondenceIndex);
		cvCorrespondenceMap.at<double>(0, correspondenceIndex) = firstPoint.x;
		cvCorrespondenceMap.at<double>(1, correspondenceIndex) = firstPoint.y;
		cvCorrespondenceMap.at<double>(2, correspondenceIndex) = secondPoint.x;
		cvCorrespondenceMap.at<double>(3, correspondenceIndex) = secondPoint.y;
	}
	return cvCorrespondenceMap;
}

std::vector<cv::Mat> EssentialMatrixDecomposition::ComputeTransformMatrix(cv::Mat fundamentalMatrix)
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

int EssentialMatrixDecomposition::FindValidTransform(std::vector<cv::Mat> projectionsList, cv::Mat correspondenceMap)
{
	static const float EPSILON = 1e-2;

	int validMatrixIndex = -1;
	bool validMatrixFound = false;
	for (unsigned matrixIndex = 0; matrixIndex < projectionsList.size(); matrixIndex++)
	{
		cv::Mat currentProjectionMatrix = projectionsList.at(matrixIndex);
		double rotationDeterminant = cv::determinant( currentProjectionMatrix( cv::Rect(0,0,2,2) ));
		bool orientationPreserved = std::abs(rotationDeterminant - 1) < EPSILON;

		bool matrixIsValid = orientationPreserved && ProjectionMatrixIsValidForTestPoints(currentProjectionMatrix, correspondenceMap);

		if (matrixIsValid)
		{
			if (validMatrixFound)
			{
				// There should only be one valid matrix. If multiple valid matrices are found, the decomposition has failed.
				return -1;
			}
			validMatrixFound = true;
			validMatrixIndex = matrixIndex;
		}
	}

	return validMatrixIndex;
}

bool EssentialMatrixDecomposition::ProjectionMatrixIsValidForTestPoints(cv::Mat projectionMatrix, cv::Mat correspondenceMap)
{
	static const float EPSILON = 1e-2;

	double rotationDeterminant = cv::determinant( projectionMatrix(cv::Rect(0,0,3,3)) );
	int rotationDeterminantSign = rotationDeterminant >= 0 ? 1 : -1;
	double principleRayX = projectionMatrix.at<double>(2, 0);
	double principleRayY = projectionMatrix.at<double>(2, 1);
	double principleRayZ = projectionMatrix.at<double>(2, 2);
	double principleRayNorm = std::sqrt (principleRayX*principleRayX + principleRayY*principleRayY + principleRayZ*principleRayZ);

	cv::Mat identityProjection(3, 4, CV_64FC1, cv::Scalar(0));
	identityProjection.at<double>(0,0) = 1;
	identityProjection.at<double>(1,1) = 1;
	identityProjection.at<double>(2,2) = 1;

	unsigned testPointsNumber = (correspondenceMap.cols > parameters.numberOfTestPoints) ? parameters.numberOfTestPoints : correspondenceMap.cols;
	cv::Mat testPointCloudMatrix;
	cv::triangulatePoints(
		firstCameraMatrix * identityProjection,
		secondCameraMatrix * projectionMatrix,
		correspondenceMap(cv::Rect(0, 0, testPointsNumber, 2 )),
		correspondenceMap(cv::Rect(0, 2, testPointsNumber, 2 )),
		testPointCloudMatrix
	);

	bool matrixIsValid = false;
	bool matrixIsInvalid = false;
	unsigned validPointsCount = 0;
	unsigned invalidPointsCount = 0;
	for (unsigned pointIndex = 0; pointIndex < testPointCloudMatrix.cols && !matrixIsValid && !matrixIsInvalid; pointIndex++)
	{
		cv::Mat testPoint = testPointCloudMatrix(cv::Rect(pointIndex, 0, 1, 4));
		cv::Mat projectedPoint = projectionMatrix * testPoint;
		float t = testPoint.at<double>(3, 0);
		float w = projectedPoint.at<double>(2, 0);
		double depth = (rotationDeterminantSign * w ) / (t * principleRayNorm);

		double scale = 4;
		cv::Mat scaledTestPoint = scale * testPoint;
		cv::Mat scaledProjectedPoint = projectionMatrix * scaledTestPoint;
		double scaledT = scaledTestPoint.at<double>(3, 0);
		double scaledW = scaledProjectedPoint.at<double>(2, 0);
		double scaledDepth = (rotationDeterminantSign * scaledW ) / (scaledT * principleRayNorm);

		//float z = testPointCloudMatrix.at<double>(2, pointIndex) / testPointCloudMatrix.at<double>(3, pointIndex);
		//bool validPoint = (z == z) && (!std::isinf(z)) && (z >= 0);
		bool validPoint = (depth < scaledDepth + EPSILON && depth > scaledDepth - EPSILON && depth > 0);
		if (validPoint)
		{
			validPointsCount++;
		}
		else
		{
			invalidPointsCount++;
		}
		matrixIsValid = validPointsCount >= testPointsNumber / 2;
		matrixIsInvalid = invalidPointsCount >= testPointsNumber / 2;
	}

	return matrixIsValid || !matrixIsInvalid;
}

void EssentialMatrixDecomposition::ValidateParameters()
{
	ASSERT(parameters.numberOfTestPoints > 0, "EssentialMatrixComputation Configuration Error: number of test points has to be positive");
	ASSERT(parameters.firstCameraMatrix.focalLengthX > 0 && parameters.firstCameraMatrix.focalLengthY > 0, "EssentialMatrixComputation Configuration Error: focalLength is not positive");
	ASSERT(parameters.secondCameraMatrix.focalLengthX > 0 && parameters.secondCameraMatrix.focalLengthY > 0, "EssentialMatrixComputation Configuration Error: focalLength is not positive");
}

void EssentialMatrixDecomposition::ValidateInputs(cv::Mat fundamentalMatrix, cv::Mat correspondenceMap)
{
}

}
}
}

/** @} */
