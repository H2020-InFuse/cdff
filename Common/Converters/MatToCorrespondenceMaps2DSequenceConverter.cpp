/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToCorrespondenceMaps2DSequenceConverter.cpp
 * @date 18/06/2018
 * @author Alessandro Bianco  and  Nassir W. Oumer 
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of MatToCorrespondenceMaps2DSequenceConverter.
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

#include "MatToCorrespondenceMaps2DSequenceConverter.hpp"
#include <Errors/Assert.hpp>
#include<iostream>

namespace Converters {

using namespace CorrespondenceMap2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

CorrespondenceMaps2DSequenceConstPtr MatToCorrespondenceMaps2DSequenceConverter::Convert(const cv::Mat&  measurementMatrix)
	{
	CorrespondenceMaps2DSequencePtr conversion = NewCorrespondenceMaps2DSequence();
	
	ASSERT( measurementMatrix.rows % 2 == 0, "MatToCorrespondenceMaps2DSequenceConverter error, measumentMatrix row number should be even");

	int numberOfImages = measurementMatrix.rows / 2;
	int numberOfPoints = measurementMatrix.cols;
	ASSERT(numberOfImages <= MAXIMUM_NUMBER_OF_IMAGES, "MatToCorrespondenceMaps2DSequenceConverter error, number of images is too large");

	for(int sourceImageIndex = 0; sourceImageIndex < numberOfImages; sourceImageIndex++)
		{
		for(int sinkImageIndex = sourceImageIndex + 1; sinkImageIndex < numberOfImages; sinkImageIndex++)
			{
			CorrespondenceMap2DPtr correspondenceMap = NewCorrespondenceMap2D();
			for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
				{
				BaseTypesWrapper::Point2D sourcePoint, sinkPoint;
				sourcePoint.x = measurementMatrix.at<float>(2*sourceImageIndex, pointIndex);
				sourcePoint.y = measurementMatrix.at<float>(2*sourceImageIndex + 1, pointIndex);
				sinkPoint.x = measurementMatrix.at<float>(2*sinkImageIndex, pointIndex);
				sinkPoint.y = measurementMatrix.at<float>(2*sinkImageIndex + 1, pointIndex);
				AddCorrespondence(*correspondenceMap, sourcePoint, sinkPoint, 1);
				}
			AddCorrespondenceMap(*conversion, *correspondenceMap);
			}
		}

	return conversion;
	}


CorrespondenceMaps2DSequenceSharedConstPtr MatToCorrespondenceMaps2DSequenceConverter::ConvertShared(const cv::Mat&  measurementMatrix)
	{
	CorrespondenceMaps2DSequenceConstPtr conversion = Convert(measurementMatrix);
	CorrespondenceMaps2DSequenceSharedConstPtr sharedConversion(conversion);
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
