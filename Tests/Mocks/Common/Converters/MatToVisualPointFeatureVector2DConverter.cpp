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

using namespace VisualPointFeatureVector2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
MatToVisualPointFeatureVector2DConverter::~MatToVisualPointFeatureVector2DConverter()
	{

	}

VisualPointFeatureVector2DConstPtr MatToVisualPointFeatureVector2DConverter::Convert(const cv::Mat& featuresMatrix)
	MOCK_METHOD(Converters::MatToVisualPointFeatureVector2DConverter, Convert, VisualPointFeatureVector2DConstPtr, (featuresMatrix) )	

}

/** @} */
