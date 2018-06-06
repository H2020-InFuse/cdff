/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "UdvDecomposition.hpp"
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

UdvDecomposition::UdvDecomposition()
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

UdvDecomposition::~UdvDecomposition()
{
}

void UdvDecomposition::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void UdvDecomposition::process()
{
	cv::Mat measurementMatrix = ComputeMeasurementMatrix(inCorrespondenceMapsSequence);
	cv::Mat centroidMatrix = ComputeMeasuresCentroid(measurementMatrix);
	CentreMeasurementMatrix(centroidMatrix, measurementMatrix);

	cv::Mat compatibleRotationMatrix, compatiblePositionMatrix;
	DecomposeMeasurementMatrix(measurementMatrix, compatibleRotationMatrix, compatiblePositionMatrix);
	
	cv::Mat rotationMatrix = ComputeRotationMatrix(compatibleRotationMatrix, centroidMatrix);

	ConvertRotationMatrixToPosesSequence(centroidMatrix, rotationMatrix, outPosesSequence);
	outSuccess = true;
}

const UdvDecomposition::UdvDecompositionOptionsSet UdvDecomposition::DEFAULT_PARAMETERS =
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

int UdvDecomposition::ComputeNumberOfImages(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence)
	{
	if ( GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) == 12 )
		{
		return 4;
		}
	else if ( GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) == 30 )
		{
		return 6;
		}
	else if ( GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) == 56 )
		{
		return 8;
		}
	else
		{
		ASSERT(false, "Unhandled number of correspondenceMaps");
		}
	return 0;
	}

cv::Mat UdvDecomposition::ComputeMeasurementMatrix(CorrespondenceMaps2DSequence& correspondenceMapsSequence)
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
			if (PointIsNotInVector(sourcePoint, usedPointsList))
				{
				continue;
				}
			//We compute a list of points that should match to the same 3D points as 'point' does in the first image
			ImagePoint sourceImagePoint(sourcePoint.x, sourcePoint.y, 0, false);
			ImagePoint sinkImagePoint(sinkPoint.x, sinkPoint.y, correspondenceIndex+1, false);
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

std::vector<UdvDecomposition::ImagePoint> UdvDecomposition::ComputeChainOfMatchingPoints(CorrespondenceMaps2DSequence& correspondenceMapsSequence, int numberOfImages, ImagePoint point1, ImagePoint point2)
	{
	std::vector<ImagePoint> chain;
	chain.push_back(point1);
	chain.push_back(point2);

	ImagePoint pointToExplore(0, 0, 0, false);
	while( ThereAreUnexploredPoints(chain, pointToExplore) )
		{
		int separationMapIndex; //after this index, all the maps treat pointToExplore.image as source, before that index they treat pointToExplore.image as sink.
		std::vector<int> mapsToExplore = ComputeMapsToExplore(numberOfImages, pointToExplore.image, separationMapIndex);
		for(std::vector<int>::iterator mapIndex = mapsToExplore.begin(); mapIndex == mapsToExplore.end(); mapIndex++)
			{
			const CorrespondenceMap2D& correspondenceMap = GetCorrespondenceMap(correspondenceMapsSequence, *mapIndex);
			for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
				{
				Point2D sourcePoint = GetSource(correspondenceMap, correspondenceIndex);
				Point2D sinkPoint = GetSink(correspondenceMap, correspondenceIndex);
				if (*mapIndex < separationMapIndex && sinkPoint.x == pointToExplore.x && sinkPoint.y == pointToExplore.y)
					{
					ImagePoint newPoint(sourcePoint.x, sourcePoint.y, GetSourceIndex(numberOfImages, *mapIndex, pointToExplore.image), false);
					chain.push_back(newPoint);
					}
				else if (*mapIndex >= separationMapIndex && sourcePoint.x == pointToExplore.x && sourcePoint.y == pointToExplore.y)
					{
					ImagePoint newPoint(sinkPoint.x, sinkPoint.y, GetSinkIndex(numberOfImages, *mapIndex, pointToExplore.image), false);
					chain.push_back(newPoint);
					}
				}
			}		
		}

	return chain;
	}

void UdvDecomposition::AddChainToMeasurementMatrix(const std::vector<ImagePoint> chain, cv::Mat& measurementMatrix)
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

void UdvDecomposition::DecomposeMeasurementMatrix(cv::Mat measurementMatrix, cv::Mat& compatibleRotationMatrix, cv::Mat& compatiblePositionMatrix)
	{
	cv::Mat leftSingularVectorsMatrix, singularValuesMatrix, rightSingularVectorsMatrix;
	cv::SVD::compute(measurementMatrix, singularValuesMatrix, leftSingularVectorsMatrix, rightSingularVectorsMatrix);

	PRINT_TO_LOG("svalue size", singularValuesMatrix.size());
	PRINT_TO_LOG("svalue type", singularValuesMatrix.type());
	ASSERT(singularValuesMatrix.rows == 3 && singularValuesMatrix.cols == 3 && singularValuesMatrix.type() == CV_32FC1, "Something is wrong");
	cv::Mat rotationMatrixColumnsList[3];
	for(int columnIndex = 0; columnIndex < 3; columnIndex++)
		{
		rotationMatrixColumnsList[columnIndex] = leftSingularVectorsMatrix(cv::Rect(columnIndex, 0, 1, leftSingularVectorsMatrix.rows)) * singularValuesMatrix.at<float>(columnIndex, columnIndex);
		}
	cv::hconcat(rotationMatrixColumnsList, 3, compatibleRotationMatrix);
	}

cv::Mat UdvDecomposition::ComputeRotationMatrix(cv::Mat compatibleRotationMatrix, cv::Mat centroidMatrix)
	{
	cv::Mat leftCameraMatrix = CameraMatrixToCvMatrix(parameters.leftCameraMatrix);
	cv::Mat rightCameraMatrix = CameraMatrixToCvMatrix(parameters.rightCameraMatrix);
	cv::Mat leftAbsoluteConicImage = leftCameraMatrix.t() * leftCameraMatrix.inv();
	cv::Mat rightAbsoluteConicImage = rightCameraMatrix.t() * rightCameraMatrix.inv();

	cv::Mat rotationMatrix;
	for(int imageIndex = 0; imageIndex < compatibleRotationMatrix.rows/2; imageIndex++)
		{
		cv::Mat compatibleCameraRotationMatrix = compatibleRotationMatrix(cv::Rect(0, 2*imageIndex, 3, 2));

		cv::Mat fullCompatibleCameraRotationMatrix;
		cv::Mat temporaryMatrixList[2] = {compatibleCameraRotationMatrix, cv::Mat(1, 3, CV_32FC1, cv::Scalar(0)) };
		cv::vconcat(temporaryMatrixList, 2, fullCompatibleCameraRotationMatrix);

		cv::Mat matrixToDecompose;
		if (imageIndex % 2 == 0)
			{
			matrixToDecompose = ( fullCompatibleCameraRotationMatrix.t() * leftAbsoluteConicImage * fullCompatibleCameraRotationMatrix).inv();
			}
		else
			{
			matrixToDecompose = ( fullCompatibleCameraRotationMatrix.t() * rightAbsoluteConicImage * fullCompatibleCameraRotationMatrix).inv();
			}
		cv::Cholesky((float*)matrixToDecompose.ptr(), matrixToDecompose.cols*sizeof(float), 3, NULL, 0, 0);
		cv::Mat metricConversionMatrix = matrixToDecompose; //MatrixToDecompose has changed.

		cv::Mat cameraRotationMatrix = compatibleCameraRotationMatrix * metricConversionMatrix;
		temporaryMatrixList[0] = rotationMatrix;
		temporaryMatrixList[0] = cameraRotationMatrix;
		cv::vconcat(temporaryMatrixList, 2, rotationMatrix);
		}

	return rotationMatrix;
	}

cv::Mat UdvDecomposition::ComputeMeasuresCentroid(cv::Mat measurementMatrix)
	{
	cv::Mat centroidMatrix(measurementMatrix.rows/2, 2, CV_32FC1);

	for(int imageIndex = 0; imageIndex < centroidMatrix.rows; imageIndex++)
		{
		float centroidX, centroidY;
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

void UdvDecomposition::CentreMeasurementMatrix(cv::Mat centroidMatrix, cv::Mat& measurementMatrix)
	{
	ASSERT( centroidMatrix.rows == measurementMatrix.rows/2, "Centroid Matrix and rotation Matrix sizes do not match");

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

void UdvDecomposition::ConvertRotationMatrixToPosesSequence(cv::Mat centroidMatrix, cv::Mat rotationMatrix, Poses3DSequence& posesSequence)
	{
	ASSERT( centroidMatrix.rows == rotationMatrix.rows/2, "Centroid Matrix and rotation Matrix sizes do not match");

	for(int poseIndex = 0; poseIndex < centroidMatrix.rows; poseIndex++)
		{
		Pose3D newPose;

		AddPose(posesSequence, newPose);
		}
	}

bool UdvDecomposition::PointIsNotInVector(Point2D point, const std::vector<Point2D>& vector)
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

void UdvDecomposition::ValidateParameters()
{
	ASSERT(parameters.leftCameraMatrix.focalLengthX > 0 && parameters.leftCameraMatrix.focalLengthY > 0, "UdvDecomposition Configuration error: left focal length has to be positive");
	ASSERT(parameters.rightCameraMatrix.focalLengthX > 0 && parameters.rightCameraMatrix.focalLengthY > 0, "UdvDecomposition Configuration error: right focal length has to be positive");
	ASSERT(parameters.baseline > 0, "UdvDecomposition Configuration error: stereo camera baseline has to be positive");
}

void UdvDecomposition::ValidateInputs()
{
	int n = GetNumberOfCorrespondenceMaps(inCorrespondenceMapsSequence);
	ASSERT( n == 12 || n == 30 || n == 56, "UdvDecomposition Error: you should provide correspondence maps for either 2, 3 or 4 pairs of stereo camera images");
}


bool UdvDecomposition::ThereAreUnexploredPoints(const std::vector<ImagePoint>& chain, ImagePoint& pointToExplore)
	{
	for(int pointIndex = 0; pointIndex < chain.size(); pointIndex++)
		{
		if (!chain.at(pointIndex).explored)
			{
			pointToExplore = chain.at(pointIndex);
			return true;
			}
		}
	return false;
	}

std::vector<int> UdvDecomposition::ComputeMapsToExplore(int numberOfImages, int imageIndex, int& separationPoint)
	{
	std::vector<int> mapsToExplore;
	bool separationPointSet = false;

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
				if (!separationPointSet)
					{
					separationPoint = mapIndex;
					}
				}
			mapIndex++;
			}
		}

	return mapsToExplore;
	}

int UdvDecomposition::GetSourceIndex(int numberOfImages, int mapIndex, int imageIndex)
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
	ASSERT(false, "UdvDecomposition::GetSourceIndex, no source index found");
	return 0;
	}

int UdvDecomposition::GetSinkIndex(int numberOfImages, int mapIndex, int imageIndex)
	{
	int startIndex = 0;
	int sourceImage = 0;
	for(sourceImage = 0; sourceImage < imageIndex; sourceImage++)
		{
		startIndex += numberOfImages - 1 - sourceImage;
		}
	ASSERT(sourceImage < imageIndex, "UdvDecomposition::GetSinkIndex, so sink index found");
	return ( mapIndex - startIndex + imageIndex + 1);
	}

bool UdvDecomposition::AllImagesAreRepresented(const std::vector<ImagePoint>& chain, int numberOfImages)
	{
	std::vector<bool> representationsList(numberOfImages);
	for(int imageIndex = 0; imageIndex < numberOfImages; imageIndex++)
		{
		representationsList.at(imageIndex) = false;
		}
	for(int pointIndex = 0; pointIndex < chain.size(); pointIndex++)
		{
		const ImagePoint& point = chain.at(pointIndex);
		ASSERT(point.image < numberOfImages, "UdvDecomposition::AllImagesAreRepresented, point.image is greated than numberOfImages");
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

cv::Mat UdvDecomposition::CameraMatrixToCvMatrix(const CameraMatrix& cameraMatrix)
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
