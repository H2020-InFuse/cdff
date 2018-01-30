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

using namespace VisualPointFeatureVector3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
const cv::Mat VisualPointFeatureVector3DToMatConverter::Convert(const VisualPointFeatureVector3DConstPtr& featuresVector)
	{	
	if (GetNumberOfPoints(*featuresVector) == 0)
		{
		return cv::Mat();
		}
	ASSERT( GetVectorType(*featuresVector) == ALL_POSITIONS_VECTOR, "VisualPointFeatureVector3DToMatConverter: non empty input feature vector must have all positions-defined points");

	int descriptorSize = GetNumberOfDescriptorComponents(*featuresVector, 0);
	cv::Mat conversion(GetNumberOfPoints(*featuresVector), 3 + descriptorSize, CV_32FC1, cv::Scalar(0));
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
		{
		conversion.at<float>(pointIndex, 0) = GetXCoordinate(*featuresVector, pointIndex);
		conversion.at<float>(pointIndex, 1) = GetYCoordinate(*featuresVector, pointIndex);
		conversion.at<float>(pointIndex, 2) = GetZCoordinate(*featuresVector, pointIndex);

		ASSERT(descriptorSize == GetNumberOfDescriptorComponents(*featuresVector, pointIndex), "VisualPointFeatureVector3DToMatConverter: Descriptors do not have the same size.");
		for(int componentIndex = 0; componentIndex < descriptorSize; componentIndex++)
			conversion.at<float>(pointIndex, componentIndex + 3) = GetDescriptorComponent(*featuresVector, pointIndex, componentIndex);
		}
	
	return conversion;
	}

const cv::Mat VisualPointFeatureVector3DToMatConverter::ConvertShared(const VisualPointFeatureVector3DSharedConstPtr& featuresVector)
	{	
	return Convert(featuresVector.get());
	}


}

/** @} */
