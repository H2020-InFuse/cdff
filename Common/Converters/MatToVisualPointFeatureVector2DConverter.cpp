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


namespace Types {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
VisualPointFeatureVector2D* MatToVisualPointFeatureVector2DConverter::Convert(cv::Mat featuresMatrix)
	{
	ASSERT( featuresMatrix.type() == CV_16UC1, "MatToVisualPointFeatureVector2DConverter: unsopported cv::mat type in input");
	ASSERT( featuresMatrix.cols == 2, "MatToVisualPointFeatureVector2DConverter: unexpected numbers of rows");

	VisualPointFeatureVector2D* conversion = new VisualPointFeatureVector2D();

	for(int rowIndex = 0; rowIndex < featuresMatrix.rows; rowIndex++)
		{
		VisualPointFeature2D* featureVector = new VisualPointFeature2D();
		featureVector->point.x = featuresMatrix.at<uint16_t>(rowIndex, 0);
		featureVector->point.y = featuresMatrix.at<uint16_t>(rowIndex, 1);	

		int error = ASN_SEQUENCE_ADD(&(conversion->list), featureVector);
		ASSERT(error == 0, "MatToVisualPointFeatureVector2DConverter, conversion failed");
		} 
	
	PRINT_TO_LOG("2", conversion->list.count);
	return conversion;
	}


}

/** @} */
