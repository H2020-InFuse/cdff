/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToCorrespondenceMaps3DSequenceConverter.cpp
 * @date 24/07/2018
 * @author Alessandro Bianco  and  Nassir W. Oumer 
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of MatToCorrespondenceMaps3DSequenceConverter.
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

#include "MatToCorrespondenceMaps3DSequenceConverter.hpp"
#include <Errors/Assert.hpp>
#include<iostream>

namespace Converters {

using namespace CorrespondenceMap3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

CorrespondenceMaps3DSequenceConstPtr MatToCorrespondenceMaps3DSequenceConverter::Convert(const cv::Mat&  measurementMatrix)
	{
	CorrespondenceMaps3DSequencePtr conversion = NewCorrespondenceMaps3DSequence();
	
	ASSERT( measurementMatrix.rows % 2 == 0, "MatToCorrespondenceMaps3DSequenceConverter error, measumentMatrix row number should be even");

	int numberOfClouds = measurementMatrix.rows / 2;
	ASSERT(numberOfClouds <= MAXIMUM_NUMBER_OF_CLOUDS, "MatToCorrespondenceMaps3DSequenceConverter error, number of clouds is too large");

	for(int sourceCloudIndex = 0; sourceCloudIndex < numberOfClouds; sourceCloudIndex++)
		{
		for(int sinkCloudIndex = sourceCloudIndex + 1; sinkCloudIndex < numberOfClouds; sinkCloudIndex++)
			{
			CorrespondenceMap3DPtr correspondenceMap = NewCorrespondenceMap3D();
			for(int pointIndex = 0; pointIndex < numberOfClouds; pointIndex++)
				{
				BaseTypesWrapper::Point3D sourcePoint, sinkPoint;
				sourcePoint.x = measurementMatrix.at<float>(2*sourceCloudIndex, pointIndex);
				sourcePoint.y = measurementMatrix.at<float>(2*sourceCloudIndex + 1, pointIndex);
				sourcePoint.z = measurementMatrix.at<float>(2*sourceCloudIndex + 2, pointIndex);
				sinkPoint.x = measurementMatrix.at<float>(2*sinkCloudIndex, pointIndex);
				sinkPoint.y = measurementMatrix.at<float>(2*sinkCloudIndex + 1, pointIndex);
				sinkPoint.z = measurementMatrix.at<float>(2*sinkCloudIndex + 2, pointIndex);
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);
				}
			AddCorrespondenceMap(*conversion, *correspondenceMap);
			}
		}

	return conversion;
	}


CorrespondenceMaps3DSequenceSharedConstPtr MatToCorrespondenceMaps3DSequenceConverter::ConvertShared(const cv::Mat&  measurementMatrix)
	{
	CorrespondenceMaps3DSequenceConstPtr conversion = Convert(measurementMatrix);
	CorrespondenceMaps3DSequenceSharedConstPtr sharedConversion(conversion);
	return sharedConversion;
	} 

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */


}

/** @} */
