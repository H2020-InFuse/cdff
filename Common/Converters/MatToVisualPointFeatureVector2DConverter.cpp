/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToVisualPointFeatureVector2DConverter.cpp
 * @date 20/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Types
 * 
 * Implementation of the Harris Detector 2D class.
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

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
CppTypes::VisualPointFeatureVector2D::ConstPtr MatToVisualPointFeatureVector2DConverter::Convert(const cv::Mat featuresMatrix)
	{
	CppTypes::VisualPointFeatureVector2D::Ptr conversion = CppTypes::VisualPointFeatureVector2D::Ptr( new CppTypes::VisualPointFeatureVector2D() );
	if (featuresMatrix.cols == 0 && featuresMatrix.rows == 0)
		return conversion;

	ASSERT( featuresMatrix.type() == CV_32FC1, "MatToVisualPointFeatureVector2DConverter: unsopported cv::mat type in input");
	ASSERT( featuresMatrix.cols >= 2, "MatToVisualPointFeatureVector2DConverter: unexpected numbers of rows");

	for(int rowIndex = 0; rowIndex < featuresMatrix.rows; rowIndex++)
		{
		conversion->AddPoint( featuresMatrix.at<float>(rowIndex, 0), featuresMatrix.at<float>(rowIndex, 1) );
		for(int columnIndex = 2; columnIndex < featuresMatrix.cols; columnIndex++)
			{
			conversion->AddDescriptorComponent(rowIndex, featuresMatrix.at<float>(rowIndex, columnIndex) );
			}
		} 

	return conversion;
	}


}

/** @} */
