/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToVisualPointFeatureVector2DConverter.cpp
 * @date 20/11/2017
 * @authors Alessandro Bianco, Xavier Martinez
 */

/*!
 * @addtogroup Types
 * 
 * Implementation of MatToVisualPointFeatureVector2DConverter class.
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

#include "MatToVisualPointFeatureVector2DConverter.hpp"
#include <Errors/Assert.hpp>


namespace Converters {

using namespace VisualPointFeatureVector2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
VisualPointFeatureVector2DConstPtr MatToVisualPointFeatureVector2DConverter::Convert(const cv::Mat& featuresMatrix)
	{
	VisualPointFeatureVector2DPtr conversion = new VisualPointFeatureVector2D();

	if (featuresMatrix.cols == 0 && featuresMatrix.rows == 0)
		return conversion;

	ASSERT( featuresMatrix.type() == CV_32FC1, "MatToVisualPointFeatureVector2DConverter: Only CV_32FC1 type is supported for this conversion for now");
	ASSERT( featuresMatrix.cols >= 2, "MatToVisualPointFeatureVector2DConverter: At least 2 rows matrixes are needed by this converter.");

	for(int rowIndex = 0; rowIndex < featuresMatrix.rows; rowIndex++)
		{
		AddPoint(*conversion, featuresMatrix.at<float>(rowIndex, 0), featuresMatrix.at<float>(rowIndex, 1) );
		for(int columnIndex = 2; columnIndex < featuresMatrix.cols; columnIndex++)
			{
			AddDescriptorComponent(*conversion, rowIndex, featuresMatrix.at<float>(rowIndex, columnIndex) );
			}
		} 

	return conversion;
	}

VisualPointFeatureVector2DSharedConstPtr MatToVisualPointFeatureVector2DConverter::ConvertShared(const cv::Mat& featuresMatrix)
	{
	VisualPointFeatureVector2DConstPtr conversion = Convert(featuresMatrix);
	VisualPointFeatureVector2DSharedConstPtr sharedConversion(conversion);
	return sharedConversion;
	}

}

/** @} */
