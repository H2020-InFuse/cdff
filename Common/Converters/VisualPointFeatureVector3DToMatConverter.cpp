/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector3DToMatConverter.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of VisualPointFeatureVector3DToMatConverter class.
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

#include "VisualPointFeatureVector3DToMatConverter.hpp"
#include <Errors/Assert.hpp>


namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
cv::Mat VisualPointFeatureVector3DToMatConverter::Convert(VisualPointFeatureVector3D* featuresVector)
	{	
	cv::Mat conversion(featuresVector->list.count, 3, CV_32FC1, cv::Scalar(0));
	for(unsigned rowIndex = 0; rowIndex < conversion.rows; rowIndex++)
		{
		VisualPointFeature3D* feature = featuresVector->list.array[rowIndex];
		conversion.at<float>(rowIndex, 0) = feature->point.x;
		conversion.at<float>(rowIndex, 1) = feature->point.y;
		conversion.at<float>(rowIndex, 2) = feature->point.z;
		}
	
	return conversion;
	}


}

/** @} */
