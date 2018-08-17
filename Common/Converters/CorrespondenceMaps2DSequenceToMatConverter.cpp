/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrespondenceMaps2DSequenceToMatConverter.cpp
 * @date 18/06/2018
 * @author Alessandro Bianco  and  Nassir W. Oumer 
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of CorrespondenceMaps2DSequenceToMatConverter.
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

#include "CorrespondenceMaps2DSequenceToMatConverter.hpp"
#include <Errors/Assert.hpp>
#include<iostream>

namespace Converters {

using namespace CorrespondenceMap2DWrapper;
using namespace BaseTypesWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

const cv::Mat CorrespondenceMaps2DSequenceToMatConverter::Convert(const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr& correspondenceMapsSequence)
	{
	return ComputeMeasurementMatrix(*correspondenceMapsSequence);
	}


const cv::Mat CorrespondenceMaps2DSequenceToMatConverter::ConvertShared(const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceSharedConstPtr& correspondenceMapsSequence)
	{
	return Convert(correspondenceMapsSequence.get());
	} 

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
const cv::Mat CorrespondenceMaps2DSequenceToMatConverter::ComputeMeasurementMatrix(const CorrespondenceMaps2DSequence& correspondenceMapsSequence)
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

std::vector<CorrespondenceMaps2DSequenceToMatConverter::ImagePoint> CorrespondenceMaps2DSequenceToMatConverter::ComputeChainOfMatchingPoints(
	const CorrespondenceMaps2DSequence& correspondenceMapsSequence, int numberOfImages, ImagePoint point1, ImagePoint point2)
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

void CorrespondenceMaps2DSequenceToMatConverter::AddChainToMeasurementMatrix(const std::vector<ImagePoint> chain, cv::Mat& measurementMatrix)
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

int CorrespondenceMaps2DSequenceToMatConverter::ComputeNumberOfImages(const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence)
	{	
	for(int candidateNumber = 0; candidateNumber <= MAXIMUM_NUMBER_OF_IMAGES; candidateNumber++)
		{
		int expectedCorrespondencesNumber = (candidateNumber * (candidateNumber - 1)) / 2;
		if (expectedCorrespondencesNumber == GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) )
			{
			return candidateNumber;
			}
		ASSERT(expectedCorrespondencesNumber < GetNumberOfCorrespondenceMaps(correspondenceMapsSequence), "CorrespondenceMaps2DSequenceToMat error, number of correspondences is not as expected");
		}
	
	ASSERT(false, "CorrespondenceMaps2DSequenceToMatConverter error, number of images is too large");
	return 0;
	}

bool CorrespondenceMaps2DSequenceToMatConverter::PointIsNotInVector(Point2D point, const std::vector<Point2D>& vector)
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

bool CorrespondenceMaps2DSequenceToMatConverter::ThereAreUnexploredPoints(const std::vector<ImagePoint>& chain, int& chainIndexToExplore)
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

std::vector<int> CorrespondenceMaps2DSequenceToMatConverter::ComputeMapsToExplore(int numberOfImages, int imageIndex, int& separationPoint)
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

int CorrespondenceMaps2DSequenceToMatConverter::GetSourceIndex(int numberOfImages, int mapIndex, int imageIndex)
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
	ASSERT(false, "CorrespondenceMaps2DSequenceToMatConverter::GetSourceIndex, no source index found");
	return 0;
	}

int CorrespondenceMaps2DSequenceToMatConverter::GetSinkIndex(int numberOfImages, int mapIndex, int imageIndex)
	{
	int startIndex = 0;
	int sourceImage = 0;
	for(sourceImage = 0; sourceImage < imageIndex; sourceImage++)
		{
		startIndex += numberOfImages - 1 - sourceImage;
		}
	return ( mapIndex - startIndex + imageIndex + 1);
	}

bool CorrespondenceMaps2DSequenceToMatConverter::AllImagesAreRepresented(const std::vector<ImagePoint>& chain, int numberOfImages)
	{
	std::vector<bool> representationsList(numberOfImages);
	for(int imageIndex = 0; imageIndex < numberOfImages; imageIndex++)
		{
		representationsList.at(imageIndex) = false;
		}
	for(int pointIndex = 0; pointIndex < chain.size(); pointIndex++)
		{
		const ImagePoint& point = chain.at(pointIndex);
		ASSERT(point.image < numberOfImages, "CorrespondenceMaps2DSequenceToMatConverter:: AllImagesAreRepresented, point.image is greated than numberOfImages");
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

bool CorrespondenceMaps2DSequenceToMatConverter::ImageIsNotInChain(const std::vector<ImagePoint>& chain, int imageIndex)
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

}

/** @} */
