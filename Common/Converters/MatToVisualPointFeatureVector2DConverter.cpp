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

using namespace CppTypes;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
VisualPointFeatureVector2D::ConstPtr MatToVisualPointFeatureVector2DConverter::Convert(const cv::Mat featuresMatrix)
	{
	VisualPointFeatureVector2D::Ptr conversion = VisualPointFeatureVector2D::Ptr( new VisualPointFeatureVector2D() );
	if (featuresMatrix.cols == 0 && featuresMatrix.rows == 0)
		return conversion;

	ASSERT( featuresMatrix.type() == CV_32FC1, "MatToVisualPointFeatureVector2DConverter: Only CV_32FC1 type is supported for this conversion for now");
	ASSERT( featuresMatrix.cols >= 2, "MatToVisualPointFeatureVector2DConverter: At least 2 rows matrixes are needed by this converter.");

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
