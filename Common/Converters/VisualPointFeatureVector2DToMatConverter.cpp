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


namespace Converters {

using namespace VisualPointFeatureVector2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
const cv::Mat VisualPointFeatureVector2DToMatConverter::Convert(const VisualPointFeatureVector2DConstPtr& featuresVector)
	{
	if (GetNumberOfPoints(*featuresVector) == 0)
		return cv::Mat();

	int descriptorSize = GetNumberOfDescriptorComponents(*featuresVector, 0);	
	cv::Mat conversion(GetNumberOfPoints(*featuresVector), 2 + descriptorSize, CV_32FC1, cv::Scalar(0));
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
		{
		conversion.at<float>(pointIndex, 0) = GetXCoordinate(*featuresVector, pointIndex);
		conversion.at<float>(pointIndex, 1) = GetYCoordinate(*featuresVector, pointIndex);

		ASSERT(descriptorSize == GetNumberOfDescriptorComponents(*featuresVector, pointIndex), "VisualPointFeatureVector2DToMatConverter: Descriptors do not have the same size.");
		for(int componentIndex = 0; componentIndex < descriptorSize; componentIndex++)
			conversion.at<float>(pointIndex, componentIndex + 2) = GetDescriptorComponent(*featuresVector, pointIndex, componentIndex);
		}
	
	return conversion;
	}
}

/** @} */
