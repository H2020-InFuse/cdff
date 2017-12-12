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

using namespace CppTypes;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
const cv::Mat VisualPointFeatureVector3DToMatConverter::Convert(const VisualPointFeatureVector3D::ConstPtr& featuresVector)
	{	
	if (featuresVector->GetNumberOfPoints() == 0)
		return cv::Mat();

	int descriptorSize = featuresVector->GetNumberOfDescriptorComponents(0);
	cv::Mat conversion(featuresVector->GetNumberOfPoints(), 3 + descriptorSize, CV_32FC1, cv::Scalar(0));
	for(int pointIndex = 0; pointIndex < featuresVector->GetNumberOfPoints(); pointIndex++)
		{
		conversion.at<float>(pointIndex, 0) = featuresVector->GetXCoordinate(pointIndex);
		conversion.at<float>(pointIndex, 1) = featuresVector->GetYCoordinate(pointIndex);
		conversion.at<float>(pointIndex, 2) = featuresVector->GetZCoordinate(pointIndex);

		ASSERT(descriptorSize == featuresVector->GetNumberOfDescriptorComponents(pointIndex), "VisualPointFeatureVector3DToMatConverter: Descriptors do not have the same size.");
		for(int componentIndex = 0; componentIndex < descriptorSize; componentIndex++)
			conversion.at<float>(pointIndex, componentIndex + 3) = featuresVector->GetDescriptorComponent(pointIndex, componentIndex);
		}
	
	return conversion;
	}


}

/** @} */
