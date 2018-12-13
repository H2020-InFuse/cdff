/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrespondenceMaps3DSequenceToMatConverter.cpp
 * @date 24/07/2018
 * @author Alessandro Bianco  and  Nassir W. Oumer 
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of CorrespondenceMaps3DSequenceToMatConverter.
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

#include "CorrespondenceMaps3DSequenceToMatConverter.hpp"
#include <Errors/Assert.hpp>
#include<iostream>

namespace Converters {

using namespace CorrespondenceMap3DWrapper;
using namespace BaseTypesWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

const cv::Mat CorrespondenceMaps3DSequenceToMatConverter::Convert(const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequenceConstPtr& correspondenceMapsSequence)
	{
	return ComputeMeasurementMatrix(*correspondenceMapsSequence);
	}


const cv::Mat CorrespondenceMaps3DSequenceToMatConverter::ConvertShared(const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequenceSharedConstPtr& correspondenceMapsSequence)
	{
	return Convert(correspondenceMapsSequence.get());
	} 

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
const cv::Mat CorrespondenceMaps3DSequenceToMatConverter::ComputeMeasurementMatrix(const CorrespondenceMaps3DSequence& correspondenceMapsSequence)
	{
	int numberOfClouds = ComputeNumberOfClouds(correspondenceMapsSequence);
	std::vector<Point3D> usedPointsList;
	cv::Mat measurementMatrix;

	//Iterating on the points detected on the first cloud, we need to consider the first N-1 maps that involve the first cloud
	for(int mapIndex = 0; mapIndex < numberOfClouds-1; mapIndex++)
		{
		const CorrespondenceMap3D& correspondenceMap = GetCorrespondenceMap(correspondenceMapsSequence, mapIndex);
		//Then for each such map we iterate on the correspondence and we examine the points of the first cloud
		for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
			{
			Point3D sourcePoint = GetSource(correspondenceMap, correspondenceIndex);
			Point3D sinkPoint = GetSink(correspondenceMap, correspondenceIndex);

			//We make sure that we do not examine the same point twice.
			if (!PointIsNotInVector(sourcePoint, usedPointsList))
				{
				continue;
				}
			//We compute a list of points that should match to the same 3D points as 'point' does in the first cloud
			CloudPoint sourceCloudPoint(sourcePoint.x, sourcePoint.y, sourcePoint.z, 0, false);
			CloudPoint sinkCloudPoint(sinkPoint.x, sinkPoint.y, sinkPoint.z, mapIndex+1, false);
			std::vector<CloudPoint> matchesChain = ComputeChainOfMatchingPoints(correspondenceMapsSequence, numberOfClouds, sourceCloudPoint, sinkCloudPoint);

			//If the chain is maximal, i.e. the 3d point is visible from all clouds, we add it to the measurement matrix
			if (matchesChain.size() == numberOfClouds && AllCloudsAreRepresented(matchesChain, numberOfClouds))
				{
				AddChainToMeasurementMatrix(matchesChain, measurementMatrix);
				usedPointsList.push_back(sourcePoint);
				}
			}
		}

	return measurementMatrix;
	}

std::vector<CorrespondenceMaps3DSequenceToMatConverter::CloudPoint> CorrespondenceMaps3DSequenceToMatConverter::ComputeChainOfMatchingPoints(
	const CorrespondenceMaps3DSequence& correspondenceMapsSequence, int numberOfClouds, CloudPoint point1, CloudPoint point2)
	{
	std::vector<CloudPoint> chain;
	chain.push_back(point1);
	chain.push_back(point2);

	int chainIndexToExplore;
	while( ThereAreUnexploredPoints(chain, chainIndexToExplore) )
		{
		CloudPoint pointToExplore = chain.at(chainIndexToExplore);
		int separationMapIndex; //before this index, all the maps treat pointToExplore.cloud as sink, after that index they treat pointToExplore.cloud as source.
		std::vector<int> mapsToExplore = ComputeMapsToExplore(numberOfClouds, pointToExplore.cloud, separationMapIndex);

		for(std::vector<int>::iterator mapIndex = mapsToExplore.begin(); mapIndex != mapsToExplore.end(); ++mapIndex)
			{
			const CorrespondenceMap3D& correspondenceMap = GetCorrespondenceMap(correspondenceMapsSequence, *mapIndex);
			for(int correspondenceIndex = 0; correspondenceIndex < GetNumberOfCorrespondences(correspondenceMap); correspondenceIndex++)
				{
				Point3D sourcePoint = GetSource(correspondenceMap, correspondenceIndex);
				Point3D sinkPoint = GetSink(correspondenceMap, correspondenceIndex);

				if (*mapIndex <= separationMapIndex && sinkPoint.x == pointToExplore.x && sinkPoint.y == pointToExplore.y && sinkPoint.z == pointToExplore.z)
					{
					int sourceCloudIndex = GetSourceIndex(numberOfClouds, *mapIndex, pointToExplore.cloud);
					if ( CloudIsNotInChain(chain, sourceCloudIndex))
						{
						CloudPoint newPoint(sourcePoint.x, sourcePoint.y, sourcePoint.z, sourceCloudIndex, false);
						chain.push_back(newPoint);
						}
					}
				else if (*mapIndex > separationMapIndex && sourcePoint.x == pointToExplore.x && sourcePoint.y == pointToExplore.y && sourcePoint.z == pointToExplore.z)
					{
					int sinkCloudIndex = GetSinkIndex(numberOfClouds, *mapIndex, pointToExplore.cloud);
					if ( CloudIsNotInChain(chain, sinkCloudIndex))
						{
						CloudPoint newPoint(sinkPoint.x, sinkPoint.y, sinkPoint.z, sinkCloudIndex, false);
						chain.push_back(newPoint);
						}
					}
				}
			}
		chain.at(chainIndexToExplore).explored = true;		
		}

	return chain;
	}

void CorrespondenceMaps3DSequenceToMatConverter::AddChainToMeasurementMatrix(const std::vector<CloudPoint>& chain, cv::Mat& measurementMatrix)
	{
	cv::Mat chainMatrix(2*chain.size(), 1, CV_32FC1);
	for(int cloudIndex = 0; cloudIndex < chain.size(); cloudIndex++)
		{
		chainMatrix.at<float>(2*cloudIndex, 0) = chain.at(cloudIndex).x;
		chainMatrix.at<float>(2*cloudIndex+1, 0) = chain.at(cloudIndex).y; 
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

int CorrespondenceMaps3DSequenceToMatConverter::ComputeNumberOfClouds(const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequence& correspondenceMapsSequence)
	{	
	for(int candidateNumber = 0; candidateNumber < MAXIMUM_NUMBER_OF_CLOUDS; candidateNumber++)
		{
		int expectedCorrespondencesNumber = (candidateNumber * (candidateNumber - 1)) / 2;
		if (expectedCorrespondencesNumber == GetNumberOfCorrespondenceMaps(correspondenceMapsSequence) )
			{
			return candidateNumber;
			}
		ASSERT(expectedCorrespondencesNumber < GetNumberOfCorrespondenceMaps(correspondenceMapsSequence), "CorrespondenceMaps3DSequenceToMat error, number of correspondences is not as expected");
		}
	
	ASSERT(false, "CorrespondenceMaps3DSequenceToMatConverter error, number of images is too large");
	return 0;
	}

bool CorrespondenceMaps3DSequenceToMatConverter::PointIsNotInVector(Point3D point, const std::vector<Point3D>& vector)
	{
	for(int pointIndex = 0; pointIndex < vector.size(); pointIndex++)
		{
		const Point3D& vectorPoint = vector.at(pointIndex);
		if (point.x == vectorPoint.x && point.y == vectorPoint.y && point.z == vectorPoint.z)
			{
			return false;
			}
		}
	return true;
	}

bool CorrespondenceMaps3DSequenceToMatConverter::ThereAreUnexploredPoints(const std::vector<CloudPoint>& chain, int& chainIndexToExplore)
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

std::vector<int> CorrespondenceMaps3DSequenceToMatConverter::ComputeMapsToExplore(int numberOfClouds, int cloudIndex, int& separationPoint)
	{
	std::vector<int> mapsToExplore;

	int mapIndex = 0;
	for(int firstCloud = 0; firstCloud < numberOfClouds; firstCloud++)
		{
		for(int secondCloud = firstCloud+1; secondCloud < numberOfClouds; secondCloud++)
			{
			if (firstCloud == cloudIndex)
				{
				mapsToExplore.push_back(mapIndex);
				}
			else if (secondCloud == cloudIndex)
				{
				mapsToExplore.push_back(mapIndex);
				separationPoint = mapIndex; //The last time the second sink image is the imageIndex, that is the separation point.
				}
			mapIndex++;
			}
		}

	return mapsToExplore;
	}

int CorrespondenceMaps3DSequenceToMatConverter::GetSourceIndex(int numberOfClouds, int mapIndex, int imageIndex)
	{
	int startIndex = 0;
	int nextStartIndex = numberOfClouds - 1;
	for(int sourceCloud = 0; sourceCloud < numberOfClouds; sourceCloud++)
		{
		if ( mapIndex >= startIndex && mapIndex < nextStartIndex)
			{
			return sourceCloud;
			}
		startIndex = nextStartIndex;
		nextStartIndex += numberOfClouds - 1 - sourceCloud;
		}
	ASSERT(false, "CorrespondenceMaps3DSequenceToMatConverter::GetSourceIndex, no source index found");
	return 0;
	}

int CorrespondenceMaps3DSequenceToMatConverter::GetSinkIndex(int numberOfClouds, int mapIndex, int imageIndex)
	{
	int startIndex = 0;
	int sourceImage = 0;
	for(sourceImage = 0; sourceImage < imageIndex; sourceImage++)
		{
		startIndex += numberOfClouds - 1 - sourceImage;
		}
	return ( mapIndex - startIndex + imageIndex + 1);
	}

bool CorrespondenceMaps3DSequenceToMatConverter::AllCloudsAreRepresented(const std::vector<CloudPoint>& chain, int numberOfClouds)
	{
	std::vector<bool> representationsList(numberOfClouds);
	for(int cloudIndex = 0; cloudIndex < numberOfClouds; cloudIndex++)
		{
		representationsList.at(cloudIndex) = false;
		}
	for(int pointIndex = 0; pointIndex < chain.size(); pointIndex++)
		{
		const CloudPoint& point = chain.at(pointIndex);
		ASSERT(point.cloud < numberOfClouds, "CorrespondenceMaps2DSequenceToMatConverter:: AllImagesAreRepresented, point.cloud is greated than numberOfImages");
		representationsList.at(point.cloud) = true;
		}
	for(int cloudIndex = 0; cloudIndex < numberOfClouds; cloudIndex++)
		{
		if (!representationsList.at(cloudIndex))
			{
			return false;
			}
		}
	return true;
	}

bool CorrespondenceMaps3DSequenceToMatConverter::CloudIsNotInChain(const std::vector<CloudPoint>& chain, int cloudIndex)
	{
	for(int pointIndex = 0; pointIndex < chain.size(); pointIndex++)
		{
		if (chain.at(pointIndex).cloud == cloudIndex)
			{
			return false;
			}
		}
	return true;
	}

}

/** @} */
