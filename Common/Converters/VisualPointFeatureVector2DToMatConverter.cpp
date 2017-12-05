/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector2DToMatConverter.cpp
 * @date 28/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Types
 * 
 * Implementation of VisualPointFeatureVector2DToMatConverter class.
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

#include "VisualPointFeatureVector2DToMatConverter.hpp"
#include <Errors/Assert.hpp>


namespace Types {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
cv::Mat VisualPointFeatureVector2DToMatConverter::Convert(VisualPointFeatureVector2D* vector)
	{	
	cv::Mat conversion(vector->list.count, 2, CV_16UC1, cv::Scalar(0));
	for(int rowIndex = 0; rowIndex < conversion.rows; rowIndex++)
		{
		VisualPointFeature2D* feature = vector->list.array[rowIndex];
		conversion.at<uint16_t>(rowIndex, 0) = feature->point.x;
		conversion.at<uint16_t>(rowIndex, 1) = feature->point.y;
		}
	
	return conversion;
	}


}

/** @} */
