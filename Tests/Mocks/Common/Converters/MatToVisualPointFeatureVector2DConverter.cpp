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

CppTypes::VisualPointFeatureVector2D::ConstPtr MatToVisualPointFeatureVector2DConverter::Convert(const cv::Mat featuresVector)
	MOCK_METHOD(Converters::MatToVisualPointFeatureVector2DConverter, Convert, CppTypes::VisualPointFeatureVector2D::ConstPtr, (featuresVector) )


}

/** @} */
