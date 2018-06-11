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
	parametersHelper.AddParameter<float>("LeftCameraMatrix", "PrinciplePointX", parameters.leftCameraMatrix.principlePointX, DEFAULT_PARAMETERS.leftCameraMatrix.principlePointX);
	parametersHelper.AddParameter<float>("LeftCameraMatrix", "PrinciplePointY", parameters.leftCameraMatrix.principlePointY, DEFAULT_PARAMETERS.leftCameraMatrix.principlePointY);

	parametersHelper.AddParameter<float>("RightCameraMatrix", "FocalLengthX", parameters.rightCameraMatrix.focalLengthX, DEFAULT_PARAMETERS.rightCameraMatrix.focalLengthX);
	parametersHelper.AddParameter<float>("RightCameraMatrix", "FocalLengthY", parameters.rightCameraMatrix.focalLengthY, DEFAULT_PARAMETERS.rightCameraMatrix.focalLengthY);
	parametersHelper.AddParameter<float>("RightCameraMatrix", "PrinciplePointX", parameters.rightCameraMatrix.principlePointX, DEFAULT_PARAMETERS.rightCameraMatrix.principlePointX);
	parametersHelper.AddParameter<float>("RightCameraMatrix", "PrinciplePointY", parameters.rightCameraMatrix.principlePointY, DEFAULT_PARAMETERS.rightCameraMatrix.principlePointY);

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
	cv::Mat measurementMatrix = ComputeMeasurementMatrix(inCorrespondenceMapsSequence);
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
		.principlePointX = 0,
		.principlePointY = 0,
	},
	.rightCameraMatrix =
	{
		.focalLengthX = 1,
		.focalLengthY = 1,
		.principlePointX = 0,
		.principlePointY = 0,
	},
	.baseline = 1.0
};

int SvdDecomposition::ComputeNumberOfImages(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence)
	{
	if ( GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) == 6 )
		{
		return 4;
		}
	else if ( GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) == 15 )
		{
		return 6;
		}
	else if ( GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) == 28 )
		{
		return 8;
		}
	else
		{
		ASSERT(false, "Unhandled number of correspondenceMaps");
		}
	return 0;
	}

cv::Mat SvdDecomposition::ComputeMeasurementMatrix(CorrespondenceMaps2DSequence& correspondenceMapsSequence)
	{
	int numberOfImages = ComputeNumberOfImages(correspondenceMapsSequence);
	std::vector<Point2D> usedPointsList;
	cv::Mat measurementMatrix;

	//Iterating on the points detected on the first image, we need to consider the first N-1 maps that involve the first image
	for(int mapIndex = 0; mapIndex < numberOfImages-1; mapIndex++)
		{
		const CorrespondenceMap2D& correspondenceMap = GetCorrespondenceMap(correspondenceMapsSequence, mapIndex);
		//Then for each such map we iterate on the correspondence and we examine the points of the first image
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
			{
			Point2D sourcePoint = GetSource(correspondenceMap, correspondenceIndex);
			Point2D sinkPoint = GetSink(correspondenceMap, correspondenceIndex);

			//We make sure that we do not examine the same point twice.
			if (!PointIsNotInVector(sourcePoint, usedPointsList))
				{
				continue;
				}
			//We compute a list of points that should match to the same 3D points as 'point' does in the first image
			ImagePoint sourceImagePoint(sourcePoint.x, sourcePoint.y, 0, false);
			ImagePoint sinkImagePoint(sinkPoint.x, sinkPoint.y, mapIndex+1, false);
			std::vector<ImagePoint> matchesChain = ComputeChainOfMatchingPoints(correspondenceMapsSequence, numberOfImages, sourceImagePoint, sinkImagePoint);

			//If the chain is maximal, i.e. the 3d point is visible from all images, we add it to the measurement matrix
			if (matchesChain.size() == numberOfImages && AllImagesAreRepresented(matchesChain, numberOfImages))
				{
				AddChainToMeasurementMatrix(matchesChain, measurementMatrix);
				usedPointsList.push_back(sourcePoint);
				}
			}
		}

	return measurementMatrix;
	}

std::vector<SvdDecomposition::ImagePoint> SvdDecomposition::ComputeChainOfMatchingPoints(CorrespondenceMaps2DSequence& correspondenceMapsSequence, int numberOfImages, ImagePoint point1, ImagePoint point2)
	{
	std::vector<ImagePoint> chain;
	chain.push_back(point1);
	chain.push_back(point2);

	int chainIndexToExplore;
	while( ThereAreUnexploredPoints(chain, chainIndexToExplore) )
		{
		ImagePoint pointToExplore = chain.at(chainIndexToExplore);
		int separationMapIndex; //before this index, all the maps treat pointToExplore.image as sink, after that index they treat pointToExplore.image as source.
		std::vector<int> mapsToExplore = ComputeMapsToExplore(numberOfImages, pointToExplore.image, separationMapIndex);

		for(std::vector<int>::iterator mapIndex = mapsToExplore.begin(); mapIndex != mapsToExplore.end(); mapIndex++)
			{
			const CorrespondenceMap2D& correspondenceMap = GetCorrespondenceMap(correspondenceMapsSequence, *mapIndex);
			for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
				{
				Point2D sourcePoint = GetSource(correspondenceMap, correspondenceIndex);
				Point2D sinkPoint = GetSink(correspondenceMap, correspondenceIndex);

				if (*mapIndex <= separationMapIndex && sinkPoint.x == pointToExplore.x && sinkPoint.y == pointToExplore.y)
					{
					int sourceImageIndex = GetSourceIndex(numberOfImages, *mapIndex, pointToExplore.image);
					if ( ImageIsNotInChain(chain, sourceImageIndex))
						{
						ImagePoint newPoint(sourcePoint.x, sourcePoint.y, sourceImageIndex, false);
						chain.push_back(newPoint);
						}
					}
				else if (*mapIndex > separationMapIndex && sourcePoint.x == pointToExplore.x && sourcePoint.y == pointToExplore.y)
					{
					int sinkImageIndex = GetSinkIndex(numberOfImages, *mapIndex, pointToExplore.image);
					if ( ImageIsNotInChain(chain, sinkImageIndex))
						{
						ImagePoint newPoint(sinkPoint.x, sinkPoint.y, sinkImageIndex, false);
						chain.push_back(newPoint);
						}
					}
				}
			}
		chain.at(chainIndexToExplore).explored = true;		
		}

	return chain;
	}

void SvdDecomposition::AddChainToMeasurementMatrix(const std::vector<ImagePoint> chain, cv::Mat& measurementMatrix)
	{
	cv::Mat chainMatrix(2*chain.size(), 1, CV_32FC1);
	for(int imageIndex = 0; imageIndex < chain.size(); imageIndex++)
		{
		chainMatrix.at<float>(2*imageIndex, 0) = chain.at(imageIndex).x;
		chainMatrix.at<float>(2*imageIndex+1, 0) = chain.at(imageIndex).y; 
		}

	if (measurementMatrix.cols == 0 && measurementMatrix.rows == 0)
		{
		measurementMatrix = chainMatrix;
		}
	else
		{
		cv::Mat matricesList[2] = {measurementMatrix, chainMatrix};
		cv::hconcat(matricesList, 2, measurementMatrix);
		}
	}

void SvdDecomposition::DecomposeMeasurementMatrix(cv::Mat measurementMatrix, cv::Mat& compatibleRotationMatrix, cv::Mat& compatiblePositionMatrix)
	{
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

	int firstPoseIndex = 0;
	cv::Mat firstProjectionMatrixColumns[4];
	firstProjectionMatrixColumns[0] = (cv::Mat_<float>(3, 1, CV_32FC1) << rotationMatrix.at<float>(2*firstPoseIndex, 0), rotationMatrix.at<float>(2*firstPoseIndex+1, 0), 0);
	firstProjectionMatrixColumns[1] = (cv::Mat_<float>(3, 1, CV_32FC1) << rotationMatrix.at<float>(2*firstPoseIndex, 1), rotationMatrix.at<float>(2*firstPoseIndex+1, 1), 0);
	firstProjectionMatrixColumns[2] = (cv::Mat_<float>(3, 1, CV_32FC1) << translationMatrix.at<float>(firstPoseIndex, 0), translationMatrix.at<float>(firstPoseIndex, 1), 1);
	firstProjectionMatrixColumns[3] = (cv::Mat_<float>(3, 1, CV_32FC1) << rotationMatrix.at<float>(2*firstPoseIndex, 2), rotationMatrix.at<float>(2*firstPoseIndex+1, 2), 0);

	cv::Mat firstProjectionMatrix;
	cv::hconcat(firstProjectionMatrixColumns, 4, firstProjectionMatrix);
	cv::Mat firstMetricRotationMatrix = ComputeMetricRotationMatrix(firstProjectionMatrix(cv::Rect(0, 0, 3, 3)), firstPoseIndex);

	cv::Mat finalRotationMatrix = rotationMatrix * firstMetricRotationMatrix;

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

		if (poseIndex == 0)
			{
			inverseOfFirstCameraTransform = affineTransform.inverse();
			}
		affineTransform = (affineTransform * inverseOfFirstCameraTransform).inverse();

		Pose3D newPose;
		SetPosition(newPose, affineTransform.translation()(0), affineTransform.translation()(1), affineTransform.translation()(2));
		Eigen::Quaternion<float> outputQuaternion( affineTransform.rotation() );
		SetOrientation(newPose, outputQuaternion.x(), outputQuaternion.y(), outputQuaternion.z(), outputQuaternion.w());

		AddPose(posesSequence, newPose);	
		}
	}

cv::Mat SvdDecomposition::ComputeMetricRotationMatrix(cv::Mat rotationMatrix, int poseIndex)
	{
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

bool SvdDecomposition::PointIsNotInVector(Point2D point, const std::vector<Point2D>& vector)
	{
	for(int pointIndex = 0; pointIndex < vector.size(); pointIndex++)
		{
		const Point2D& vectorPoint = vector.at(pointIndex);
		if (point.x == vectorPoint.x && point.y == vectorPoint.y)
			{
			return false;
			}
		}
	return true;
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


bool SvdDecomposition::ThereAreUnexploredPoints(const std::vector<ImagePoint>& chain, int& chainIndexToExplore)
	{
	for(int pointIndex = 0; pointIndex < chain.size(); pointIndex++)
		{
		if (!chain.at(pointIndex).explored)
			{
			chainIndexToExplore = pointIndex;
			return true;
			}
		}
	return false;
	}

std::vector<int> SvdDecomposition::ComputeMapsToExplore(int numberOfImages, int imageIndex, int& separationPoint)
	{
	std::vector<int> mapsToExplore;

	int mapIndex = 0;
	for(int firstImage = 0; firstImage < numberOfImages; firstImage++)
		{
		for(int secondImage = firstImage+1; secondImage < numberOfImages; secondImage++)
			{
			if (firstImage == imageIndex)
				{
				mapsToExplore.push_back(mapIndex);
				}
			else if (secondImage == imageIndex)
				{
				mapsToExplore.push_back(mapIndex);
				separationPoint = mapIndex; //The last time the second sink image is the imageIndex, that is the separation point.
				}
			mapIndex++;
			}
		}

	return mapsToExplore;
	}

int SvdDecomposition::GetSourceIndex(int numberOfImages, int mapIndex, int imageIndex)
	{
	int startIndex = 0;
	int nextStartIndex = numberOfImages - 1;
	for(int sourceImage = 0; sourceImage < numberOfImages; sourceImage++)
		{
		if ( mapIndex >= startIndex && mapIndex < nextStartIndex)
			{
			return sourceImage;
			}
		startIndex = nextStartIndex;
		nextStartIndex += numberOfImages - 1 - sourceImage;
		}
	ASSERT(false, "SvdDecomposition::GetSourceIndex, no source index found");
	return 0;
	}

int SvdDecomposition::GetSinkIndex(int numberOfImages, int mapIndex, int imageIndex)
	{
	int startIndex = 0;
	int sourceImage = 0;
	for(sourceImage = 0; sourceImage < imageIndex; sourceImage++)
		{
		startIndex += numberOfImages - 1 - sourceImage;
		}
	return ( mapIndex - startIndex + imageIndex + 1);
	}

bool SvdDecomposition::AllImagesAreRepresented(const std::vector<ImagePoint>& chain, int numberOfImages)
	{
	std::vector<bool> representationsList(numberOfImages);
	for(int imageIndex = 0; imageIndex < numberOfImages; imageIndex++)
		{
		representationsList.at(imageIndex) = false;
		}
	for(int pointIndex = 0; pointIndex < chain.size(); pointIndex++)
		{
		const ImagePoint& point = chain.at(pointIndex);
		ASSERT(point.image < numberOfImages, "SvdDecomposition::AllImagesAreRepresented, point.image is greated than numberOfImages");
		representationsList.at(point.image) = true;
		}
	for(int imageIndex = 0; imageIndex < numberOfImages; imageIndex++)
		{
		if (!representationsList.at(imageIndex))
			{
			return false;
			}
		}
	return true;
	}

bool SvdDecomposition::ImageIsNotInChain(const std::vector<ImagePoint>& chain, int imageIndex)
	{
	for(int pointIndex = 0; pointIndex < chain.size(); pointIndex++)
		{
		if (chain.at(pointIndex).image == imageIndex)
			{
			return false;
			}
		}
	return true;
	}

cv::Mat SvdDecomposition::CameraMatrixToCvMatrix(const CameraMatrix& cameraMatrix)
	{
	cv::Mat cvCameraMatrix(3, 3, CV_32FC1, cv::Scalar(0));
	cvCameraMatrix.at<float>(0,0) = cameraMatrix.focalLengthX;
	cvCameraMatrix.at<float>(1,1) = cameraMatrix.focalLengthY;
	cvCameraMatrix.at<float>(2,0) = cameraMatrix.principlePointX;
	cvCameraMatrix.at<float>(2,1) = cameraMatrix.principlePointY;
	cvCameraMatrix.at<float>(2,2) = 1.0;

	return cvCameraMatrix;
	}
}

/** @} */
