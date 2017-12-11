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
	if (featuresVector->GetNumberOfPoints() == 0)
		return cv::Mat();

	int descriptorSize = featuresVector->GetNumberOfDescriptorComponents(0);	
	cv::Mat conversion(featuresVector->GetNumberOfPoints(), 2 + descriptorSize, CV_32FC1, cv::Scalar(0));
	for(int pointIndex = 0; pointIndex < featuresVector->GetNumberOfPoints(); pointIndex++)
		{
		conversion.at<float>(pointIndex, 0) = featuresVector->GetXCoordinate(pointIndex);
		conversion.at<float>(pointIndex, 1) = featuresVector->GetYCoordinate(pointIndex);

		ASSERT(descriptorSize == featuresVector->GetNumberOfDescriptorComponents(pointIndex), "Descriptors do not have the same size in VisualPointFeatureVector2D");
		for(int componentIndex = 0; componentIndex < descriptorSize; componentIndex++)
			conversion.at<float>(pointIndex, componentIndex + 2) = featuresVector->GetDescriptorComponent(pointIndex, componentIndex);
		}
	
	return conversion;
	}


}

/** @} */
