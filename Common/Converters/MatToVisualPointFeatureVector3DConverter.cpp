/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToVisualPointFeatureVector3DConverter.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of MatToVisualPointFeatureVector3DConverter class.
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

#include "MatToVisualPointFeatureVector3DConverter.hpp"
#include <Errors/Assert.hpp>


namespace Converters {

using namespace VisualPointFeatureVector3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
VisualPointFeatureVector3DConstPtr MatToVisualPointFeatureVector3DConverter::Convert(const cv::Mat& featuresMatrix)
	{
	VisualPointFeatureVector3DPtr conversion =  std::make_shared<VisualPointFeatureVector3D>();
	Convert(featuresMatrix, *conversion);
	return conversion;
	}

void MatToVisualPointFeatureVector3DConverter::Convert(const cv::Mat& featuresMatrix, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& conversion)
	{
	if (featuresMatrix.cols == 0 && featuresMatrix.rows == 0)
		return;

	ASSERT( featuresMatrix.type() == CV_32FC1, "MatToVisualPointFeatureVector3DConverter: Only CV_32FC1 type is supported for this conversion for now");
	ASSERT( featuresMatrix.cols >= 3, "MatToVisualPointFeatureVector3DConverter: At least 3 rows matrixes are needed by this converter.");

	for(int rowIndex = 0; rowIndex < featuresMatrix.rows; rowIndex++)
		{
		AddPoint(conversion, featuresMatrix.at<float>(rowIndex, 0), featuresMatrix.at<float>(rowIndex, 1), featuresMatrix.at<float>(rowIndex, 2) );
		for(int columnIndex = 3; columnIndex < featuresMatrix.cols; columnIndex++)
			{
			AddDescriptorComponent(conversion, rowIndex, featuresMatrix.at<float>(rowIndex, columnIndex) );
			}
		} 
	}


}

/** @} */
