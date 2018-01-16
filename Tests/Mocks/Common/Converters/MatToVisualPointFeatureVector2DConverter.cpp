/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatToVisualPointFeatureVector2DConverter.cpp
 * @date 21/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of the MatToVisualPointFeatureVector2DConverter class.
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
#include "Mocks/MockMacro.hpp"

namespace Mocks {


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
MatToVisualPointFeatureVector2DConverter::~MatToVisualPointFeatureVector2DConverter()
	{

	}

VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr MatToVisualPointFeatureVector2DConverter::Convert(const cv::Mat& featuresVector)
	MOCK_METHOD(Converters::MatToVisualPointFeatureVector2DConverter, Convert, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr, (featuresVector) )

void MatToVisualPointFeatureVector2DConverter::Convert(const cv::Mat& featuresVector, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& conversion)
	MOCK_VOID_METHOD(Converters::MatToVisualPointFeatureVector2DConverter, Convert, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D, (featuresVector), (conversion) )	

}

/** @} */
