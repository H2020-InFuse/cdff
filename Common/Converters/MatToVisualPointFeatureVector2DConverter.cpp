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
	ASSERT( featuresMatrix.type() == CV_16UC1, "MatToVisualPointFeatureVector2DConverter: unsopported cv::mat type in input");
	ASSERT( featuresMatrix.cols == 2, "MatToVisualPointFeatureVector2DConverter: unexpected numbers of rows");

	CppTypes::VisualPointFeatureVector2D::Ptr conversion = CppTypes::VisualPointFeatureVector2D::Ptr( new CppTypes::VisualPointFeatureVector2D() );

	for(int rowIndex = 0; rowIndex < featuresMatrix.rows; rowIndex++)
		{
		conversion->AddPoint( featuresMatrix.at<uint16_t>(rowIndex, 0), featuresMatrix.at<uint16_t>(rowIndex, 1) );
		} 

	return conversion;
	}


}

/** @} */
