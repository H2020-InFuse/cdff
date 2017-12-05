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

VisualPointFeatureVector2D* MatToVisualPointFeatureVector2DConverter::Convert(cv::Mat featuresVector)
	MOCK_METHOD(Types::MatToVisualPointFeatureVector2DConverter, Convert, VisualPointFeatureVector2D*, (featuresVector) )


}

/** @} */
