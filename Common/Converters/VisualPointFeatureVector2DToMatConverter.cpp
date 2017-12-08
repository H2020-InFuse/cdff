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


namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
const cv::Mat VisualPointFeatureVector2DToMatConverter::Convert(CppTypes::VisualPointFeatureVector2D::ConstPtr featuresVector)
	{	
	cv::Mat conversion(featuresVector->GetNumberOfPoints(), 2, CV_16UC1, cv::Scalar(0));
	for(int pointIndex = 0; pointIndex < featuresVector->GetNumberOfPoints(); pointIndex++)
		{
		conversion.at<uint16_t>(pointIndex, 0) = featuresVector->GetXCoordinate(pointIndex);
		conversion.at<uint16_t>(pointIndex, 1) = featuresVector->GetYCoordinate(pointIndex);
		}
	
	return conversion;
	}


}

/** @} */
