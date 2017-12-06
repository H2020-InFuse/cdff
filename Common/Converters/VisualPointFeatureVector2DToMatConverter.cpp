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
 * @remarks Returns a type using smart pointers
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
		cv::Mat conversion(vector->nCount, 2, CV_16UC1, cv::Scalar(0));
		for(unsigned rowIndex = 0; rowIndex < conversion.rows; rowIndex++)
		{
			VisualPointFeature2D feature = vector->arr[rowIndex];
			conversion.at<uint16_t>(rowIndex, 0) = feature.point.x;
			conversion.at<uint16_t>(rowIndex, 1) = feature.point.y;
		}
	
	return conversion;
	}
}

/** @} */
