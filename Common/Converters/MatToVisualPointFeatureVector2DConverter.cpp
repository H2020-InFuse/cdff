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


namespace Types {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
VisualPointFeatureVector2D* MatToVisualPointFeatureVector2DConverter::Convert(cv::Mat featuresMatrix)
	{
	ASSERT( featuresMatrix.type() == CV_16UC1, "MatToVisualPointFeatureVector2DConverter: Only CV_16UC1 type is supported for this conversion.");
	ASSERT( featuresMatrix.cols == 2, "MatToVisualPointFeatureVector2DConverter: Only 2 rows matrixes are supported by this converter.");
	ASSERT( featuresMatrix.rows * featuresMatrix.cols <= featuresElementsMax, "MatToVisualPointFeatureVector2DConverter: VisualPointFeature2D does not handle as many features. It is limited to " +featuresElementsMax+" items" );
	
	VisualPointFeatureVector2D* conversion = new VisualPointFeatureVector2D();
	VisualPointFeatureVector2D_Initialize(conversion);
	for(signed int rowIndex = 0; rowIndex < featuresMatrix.rows; rowIndex++)
		{
		  VisualPointFeature2D featureVector;
		  VisualPointFeature2D_Initialize(&featureVector);		  
		  VisualPointFeature2D_descriptor descriptor;
		  VisualPointFeature2D_descriptor_Initialize(&descriptor);
		  
		  featureVector.point.x = featuresMatrix.at<uint16_t>(rowIndex, 0);
		  featureVector.point.y = featuresMatrix.at<uint16_t>(rowIndex, 1);
		  conversion->arr[rowIndex] = featureVector;
		  conversion->nCount +=1;
		} 
	return conversion;
	}
}

/** @} */
