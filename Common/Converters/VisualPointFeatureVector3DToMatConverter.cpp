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
const cv::Mat VisualPointFeatureVector3DToMatConverter::Convert(CppTypes::VisualPointFeatureVector3D::ConstPtr featuresVector)
	{	
	cv::Mat conversion(featuresVector->GetNumberOfPoints(), 3, CV_32FC1, cv::Scalar(0));
	for(int pointIndex = 0; pointIndex < featuresVector->GetNumberOfPoints(); pointIndex++)
		{
		conversion.at<float>(pointIndex, 0) = featuresVector->GetXCoordinate(pointIndex);
		conversion.at<float>(pointIndex, 1) = featuresVector->GetYCoordinate(pointIndex);
		conversion.at<float>(pointIndex, 2) = featuresVector->GetZCoordinate(pointIndex);
		}
	
	return conversion;
	}


}

/** @} */
