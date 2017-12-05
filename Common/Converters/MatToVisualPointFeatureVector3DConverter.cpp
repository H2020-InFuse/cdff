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
VisualPointFeatureVector3D* MatToVisualPointFeatureVector3DConverter::Convert(cv::Mat featuresMatrix)
	{
	ASSERT( featuresMatrix.type() == CV_32FC1, "MatToVisualPointFeatureVector3DConverter: unsopported cv::mat type in input");
	ASSERT( featuresMatrix.cols == 3, "MatToVisualPointFeatureVector3DConverter: unexpected numbers of rows");

	VisualPointFeatureVector3D* conversion = new VisualPointFeatureVector3D();

	for(int rowIndex = 0; rowIndex < featuresMatrix.rows; rowIndex++)
		{
		VisualPointFeature3D* featureVector = new VisualPointFeature3D();
		featureVector->point.x = featuresMatrix.at<float>(rowIndex, 0);
		featureVector->point.y = featuresMatrix.at<float>(rowIndex, 1);	
		featureVector->point.z = featuresMatrix.at<float>(rowIndex, 2);	

		int error = ASN_SEQUENCE_ADD(&(conversion->list), featureVector);
		ASSERT(error == 0, "MatToVisualPointFeatureVector2DConverter, conversion failed");
		} 
	
	return conversion;
	}


}

/** @} */
