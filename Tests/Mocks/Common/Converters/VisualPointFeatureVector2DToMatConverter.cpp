/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector2DToMatConverter.cpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of the VisualPointFeatureVector2DToMatConverter class.
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
#include "VisualPointFeatureVector2DToMatConverter.hpp"
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
VisualPointFeatureVector2DToMatConverter::~VisualPointFeatureVector2DToMatConverter()
	{

	}

const cv::Mat VisualPointFeatureVector2DToMatConverter::Convert(const VisualPointFeatureVector2DConstPtr& featuresVector)
	MOCK_METHOD(Converters::VisualPointFeatureVector2DToMatConverter, Convert, cv::Mat, (featuresVector) )	

}

/** @} */
