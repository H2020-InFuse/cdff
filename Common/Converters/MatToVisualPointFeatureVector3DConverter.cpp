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

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
CppTypes::VisualPointFeatureVector3D::ConstPtr MatToVisualPointFeatureVector3DConverter::Convert(const cv::Mat featuresMatrix)
	{
	ASSERT( featuresMatrix.type() == CV_32FC1, "MatToVisualPointFeatureVector3DConverter: unsopported cv::mat type in input");
	ASSERT( featuresMatrix.cols == 3, "MatToVisualPointFeatureVector3DConverter: unexpected numbers of rows");

	CppTypes::VisualPointFeatureVector3D::Ptr conversion = CppTypes::VisualPointFeatureVector3D::Ptr( new CppTypes::VisualPointFeatureVector3D() );

	for(int rowIndex = 0; rowIndex < featuresMatrix.rows; rowIndex++)
		{
		conversion->AddPoint(featuresMatrix.at<float>(rowIndex, 0), featuresMatrix.at<float>(rowIndex, 1), featuresMatrix.at<float>(rowIndex, 2) );
		} 
	
	return conversion;
	}


}

/** @} */
