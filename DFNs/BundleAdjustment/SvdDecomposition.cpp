/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "SvdDecomposition.hpp"
#include <FrameToMatConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <fstream>

using namespace Helpers;
using namespace Converters;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace BaseTypesWrapper;

namespace dfn_ci
{

SvdDecomposition::SvdDecomposition()
{
	parametersHelper.AddParameter<float>("LeftCameraMatrix", "FocalLengthX", parameters.leftCameraMatrix.focalLengthX, DEFAULT_PARAMETERS.leftCameraMatrix.focalLengthX);
	parametersHelper.AddParameter<float>("LeftCameraMatrix", "FocalLengthY", parameters.leftCameraMatrix.focalLengthY, DEFAULT_PARAMETERS.leftCameraMatrix.focalLengthY);
	parametersHelper.AddParameter<float>("LeftCameraMatrix", "PrincipalPointX", parameters.leftCameraMatrix.principalPointX, DEFAULT_PARAMETERS.leftCameraMatrix.principalPointX);
	parametersHelper.AddParameter<float>("LeftCameraMatrix", "PrincipalPointY", parameters.leftCameraMatrix.principalPointY, DEFAULT_PARAMETERS.leftCameraMatrix.principalPointY);

	parametersHelper.AddParameter<float>("RightCameraMatrix", "FocalLengthX", parameters.rightCameraMatrix.focalLengthX, DEFAULT_PARAMETERS.rightCameraMatrix.focalLengthX);
	parametersHelper.AddParameter<float>("RightCameraMatrix", "FocalLengthY", parameters.rightCameraMatrix.focalLengthY, DEFAULT_PARAMETERS.rightCameraMatrix.focalLengthY);
	parametersHelper.AddParameter<float>("RightCameraMatrix", "PrincipalPointX", parameters.rightCameraMatrix.principalPointX, DEFAULT_PARAMETERS.rightCameraMatrix.principalPointX);
	parametersHelper.AddParameter<float>("RightCameraMatrix", "PrincipalPointY", parameters.rightCameraMatrix.principalPointY, DEFAULT_PARAMETERS.rightCameraMatrix.principalPointY);

	parametersHelper.AddParameter<float>("GeneralParameters", "Baseline", parameters.baseline, DEFAULT_PARAMETERS.baseline);

	configurationFilePath = "";
}

SvdDecomposition::~SvdDecomposition()
{
}

void SvdDecomposition::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

	leftCameraMatrix = CameraMatrixToCvMatrix(parameters.leftCameraMatrix);
	rightCameraMatrix = CameraMatrixToCvMatrix(parameters.rightCameraMatrix);
	leftCameraMatrixInverse = leftCameraMatrix.inv();
	rightCameraMatrixInverse = rightCameraMatrix.inv();
	leftAbsoluteConicImage = leftCameraMatrix.t() * leftCameraMatrixInverse;
	rightAbsoluteConicImage = rightCameraMatrix.t() * rightCameraMatrixInverse;
}

void SvdDecomposition::process()
{
	cv::Mat measurementMatrix = framesSequenceConverter.Convert(&inCorrespondenceMapsSequence);
	if (measurementMatrix.cols < 4) //Not enough points available.
		{
		outSuccess = false;
		return;
		}

	cv::Mat centroidMatrix = ComputeMeasuresCentroid(measurementMatrix);
	cv::Mat translationMatrix = ComputeTranslationMatrix(centroidMatrix);

	CentreMeasurementMatrix(centroidMatrix, measurementMatrix);
	cv::Mat compatibleRotationMatrix, compatiblePositionMatrix;
	DecomposeMeasurementMatrix(measurementMatrix, compatibleRotationMatrix, compatiblePositionMatrix);

	ConvertRotationTranslationMatricesToPosesSequence(translationMatrix, compatibleRotationMatrix, outPosesSequence);
	outSuccess = true;
}

const SvdDecomposition::SvdDecompositionOptionsSet SvdDecomposition::DEFAULT_PARAMETERS =
{
	.leftCameraMatrix =
	{
		.focalLengthX = 1,
		.focalLengthY = 1,
		.principalPointX = 0,
		.principalPointY = 0,
	},
	.rightCameraMatrix =
	{
		.focalLengthX = 1,
		.focalLengthY = 1,
		.principalPointX = 0,
		.principalPointY = 0,
	},
	.baseline = 1.0
};

void SvdDecomposition::DecomposeMeasurementMatrix(cv::Mat measurementMatrix, cv::Mat& compatibleRotationMatrix, cv::Mat& compatiblePositionMatrix)
	{
	//This is the algorithm of Tomasi and Kanade as describe in page 437 of "Multiple View Geometry in Computer Vision" by Richard Hartley, and Andrew Zisserman.
	cv::Mat leftSingularVectorsMatrix, singularValuesMatrix, rightSingularVectorsMatrix;
	cv::SVD::compute(measurementMatrix, singularValuesMatrix, leftSingularVectorsMatrix, rightSingularVectorsMatrix);

	ASSERT(singularValuesMatrix.cols == 1 && singularValuesMatrix.rows == measurementMatrix.cols && singularValuesMatrix.type() == CV_32FC1, "Something went wrong in the svd decomposition");
	ASSERT(leftSingularVectorsMatrix.rows == measurementMatrix.rows, "Unexpected result in svd decomposition");
	cv::Mat rotationMatrixColumnsList[3];
	for(int columnIndex = 0; columnIndex < 3; columnIndex++)
		{
		rotationMatrixColumnsList[columnIndex] = leftSingularVectorsMatrix(cv::Rect(columnIndex, 0, 1, leftSingularVectorsMatrix.rows)) * singularValuesMatrix.at<float>(columnIndex, 0);
		}
	cv::hconcat(rotationMatrixColumnsList, 3, compatibleRotationMatrix);

	compatiblePositionMatrix = rightSingularVectorsMatrix( cv::Rect(0, 0, rightSingularVectorsMatrix.cols, 3) );
	}

cv::Mat SvdDecomposition::ComputeTranslationMatrix(cv::Mat centroidMatrix)
	{
	//the centroid matrix contains the translation direction vectors. In order to find the vector expressed in meters we use the measure of the baseline provided as an input parameter.
	float estimatedBaselineX = centroidMatrix.at<float>(0, 0) - centroidMatrix.at<float>(1, 0);
	float estimatedBaselineY = centroidMatrix.at<float>(0, 1) - centroidMatrix.at<float>(1, 1);
	float estimatedBaseline = std::sqrt( estimatedBaselineX*estimatedBaselineX + estimatedBaselineY*estimatedBaselineY);
	float metricCoefficient = parameters.baseline / estimatedBaseline;

	cv::Mat temporaryImagesList[2] = {centroidMatrix, cv::Mat::ones(centroidMatrix.rows, 1, CV_32FC1)};
	cv::Mat translationMatrix;
	cv::hconcat(temporaryImagesList, 2, translationMatrix);

	translationMatrix = translationMatrix * metricCoefficient;
	return translationMatrix;
	}

cv::Mat SvdDecomposition::ComputeMeasuresCentroid(cv::Mat measurementMatrix)
	{
	cv::Mat centroidMatrix(measurementMatrix.rows/2, 2, CV_32FC1);

	for(int imageIndex = 0; imageIndex < centroidMatrix.rows; imageIndex++)
		{
		float centroidX = 0;
		float centroidY = 0;
		for(int measureIndex = 0; measureIndex < measurementMatrix.cols; measureIndex++)
			{
			centroidX += measurementMatrix.at<float>(2*imageIndex, measureIndex);
			centroidY += measurementMatrix.at<float>(2*imageIndex+1, measureIndex);	
			}
		centroidX /= (float)measurementMatrix.cols;
		centroidY /= (float)measurementMatrix.cols;
		centroidMatrix.at<float>(imageIndex, 0) = centroidX;
		centroidMatrix.at<float>(imageIndex, 1) = centroidY;
		}
		
	return centroidMatrix;
	}

void SvdDecomposition::CentreMeasurementMatrix(cv::Mat centroidMatrix, cv::Mat& measurementMatrix)
	{
	ASSERT( centroidMatrix.rows == measurementMatrix.rows/2, "Centroid Matrix and measurement Matrix sizes do not match");

	for(int imageIndex = 0; imageIndex < centroidMatrix.rows; imageIndex++)
		{
		float centroidX = centroidMatrix.at<float>(imageIndex, 0);
		float centroidY = centroidMatrix.at<float>(imageIndex, 1);
		for(int measureIndex = 0; measureIndex < measurementMatrix.cols; measureIndex++)
			{
			measurementMatrix.at<float>(2*imageIndex, measureIndex) -= centroidX;
			measurementMatrix.at<float>(2*imageIndex+1, measureIndex) -= centroidY;
			}
		}
	}

void SvdDecomposition::ConvertRotationTranslationMatricesToPosesSequence(cv::Mat translationMatrix, cv::Mat rotationMatrix, PoseWrapper::Poses3DSequence& posesSequence)
	{
	ASSERT( translationMatrix.rows == rotationMatrix.rows/2, "Translation Matrix and rotation Matrix sizes do not match");

	// the rotation matrix is an affine rotation matrix, it needs to be multipled on the right by an appropriate 3x3 matrix.
	// ComputeMetricRotationMatrix finds the matrix according to section 10.4.2 of "Multiple View Geometry in Computer Vision" by Richard Hartley, and Andrew Zisserman.
	cv::Mat finalRotationMatrix, firstMetricRotationMatrix;
		{	
		// x = M X + t for the first camera matrix, M is given by the first two row of the rotation matrix and t is given by the first centroid;
		// The rotation matrix of the projective 3x4 transformation matrix P = [M'|m]. M' is given by the first two columns of M concatenated with t, concatenated with a final row (0, 0, 1).  
		int firstPoseIndex = 0;
		cv::Mat firstProjectionMatrixColumns[4];
		firstProjectionMatrixColumns[0] = (cv::Mat_<float>(3, 1, CV_32FC1) << rotationMatrix.at<float>(2*firstPoseIndex, 0), rotationMatrix.at<float>(2*firstPoseIndex+1, 0), 0);
		firstProjectionMatrixColumns[1] = (cv::Mat_<float>(3, 1, CV_32FC1) << rotationMatrix.at<float>(2*firstPoseIndex, 1), rotationMatrix.at<float>(2*firstPoseIndex+1, 1), 0);
		firstProjectionMatrixColumns[2] = (cv::Mat_<float>(3, 1, CV_32FC1) << translationMatrix.at<float>(firstPoseIndex, 0), translationMatrix.at<float>(firstPoseIndex, 1), 1);
		firstProjectionMatrixColumns[3] = (cv::Mat_<float>(3, 1, CV_32FC1) << rotationMatrix.at<float>(2*firstPoseIndex, 2), rotationMatrix.at<float>(2*firstPoseIndex+1, 2), 0);

		cv::Mat firstProjectionMatrix;
		cv::hconcat(firstProjectionMatrixColumns, 4, firstProjectionMatrix);
		firstMetricRotationMatrix = ComputeMetricRotationMatrix(firstProjectionMatrix(cv::Rect(0, 0, 3, 3)), firstPoseIndex);

		finalRotationMatrix = rotationMatrix * firstMetricRotationMatrix;
		}

	AffineTransform inverseOfFirstCameraTransform;
	for(int poseIndex = 0; poseIndex < translationMatrix.rows; poseIndex++)
		{
		Eigen::Matrix3f eigenRotationMatrix;
		eigenRotationMatrix << finalRotationMatrix.at<float>(2*poseIndex, 0), finalRotationMatrix.at<float>(2*poseIndex, 1), finalRotationMatrix.at<float>(2*poseIndex, 2),
			finalRotationMatrix.at<float>(2*poseIndex+1, 0), finalRotationMatrix.at<float>(2*poseIndex+1, 1), finalRotationMatrix.at<float>(2*poseIndex+1, 2),
			0, 0, 0;
		Eigen::Quaternion<float> eigenRotation(eigenRotationMatrix);
		eigenRotation.normalize();
		Eigen::Translation<float, 3> translation(translationMatrix.at<float>(poseIndex,0), translationMatrix.at<float>(poseIndex,1), translationMatrix.at<float>(poseIndex, 2));
		AffineTransform affineTransform = translation * eigenRotation;

		// We put the transforms in the reference frame of the first camera.
		if (poseIndex == 0)
			{
			inverseOfFirstCameraTransform = affineTransform.inverse();
			}
		affineTransform = affineTransform * inverseOfFirstCameraTransform;
		AffineTransform cameraTransform = affineTransform.inverse();

		Pose3D newPose;
		SetPosition(newPose, cameraTransform.translation()(0), cameraTransform.translation()(1), cameraTransform.translation()(2));
		Eigen::Quaternion<float> outputQuaternion( cameraTransform.rotation() );
		SetOrientation(newPose, outputQuaternion.x(), outputQuaternion.y(), outputQuaternion.z(), outputQuaternion.w());

		PRINT_TO_LOG("newPose", ToString(newPose));
		AddPose(posesSequence, newPose);	
		}
	}

cv::Mat SvdDecomposition::ComputeMetricRotationMatrix(cv::Mat rotationMatrix, int poseIndex)
	{
	// computation of the metric matrix according to section 10.4.2 of "Multiple View Geometry in Computer Vision" by Richard Hartley, and Andrew Zisserman.
	ASSERT(rotationMatrix.cols == 3 && rotationMatrix.rows == 3 && rotationMatrix.type() == CV_32FC1, "unexpected rotation matrix format");
	cv::Mat matrixToDecompose;
	if (poseIndex % 2 == 0)
		{
		matrixToDecompose = ( rotationMatrix.t() * leftAbsoluteConicImage * rotationMatrix).inv();
		}
	else
		{
		matrixToDecompose = ( rotationMatrix.t() * rightAbsoluteConicImage * rotationMatrix).inv();
		}
	cv::Cholesky((float*)matrixToDecompose.ptr(), matrixToDecompose.cols*sizeof(float), 3, NULL, 0, 0);
	return matrixToDecompose; 		
	}

void SvdDecomposition::ValidateParameters()
{
	ASSERT(parameters.leftCameraMatrix.focalLengthX > 0 && parameters.leftCameraMatrix.focalLengthY > 0, "SvdDecomposition Configuration error: left focal length has to be positive");
	ASSERT(parameters.rightCameraMatrix.focalLengthX > 0 && parameters.rightCameraMatrix.focalLengthY > 0, "SvdDecomposition Configuration error: right focal length has to be positive");
	ASSERT(parameters.baseline > 0, "SvdDecomposition Configuration error: stereo camera baseline has to be positive");
}

void SvdDecomposition::ValidateInputs()
{
	int n = GetNumberOfCorrespondenceMaps(inCorrespondenceMapsSequence);
	ASSERT( n == 6 || n == 15 || n == 28, "SvdDecomposition Error: you should provide correspondence maps for either 2, 3 or 4 pairs of stereo camera images");
}

cv::Mat SvdDecomposition::CameraMatrixToCvMatrix(const CameraMatrix& cameraMatrix)
	{
	cv::Mat cvCameraMatrix(3, 3, CV_32FC1, cv::Scalar(0));
	cvCameraMatrix.at<float>(0,0) = cameraMatrix.focalLengthX;
	cvCameraMatrix.at<float>(1,1) = cameraMatrix.focalLengthY;
	cvCameraMatrix.at<float>(2,0) = cameraMatrix.principalPointX;
	cvCameraMatrix.at<float>(2,1) = cameraMatrix.principalPointY;
	cvCameraMatrix.at<float>(2,2) = 1.0;

	return cvCameraMatrix;
	}
}

/** @} */
